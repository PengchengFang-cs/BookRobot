from transitions.extensions import GraphMachine
import threading
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional

from teleoperation.p2p_planner                  import p2pPlanner
from teleoperation.traj_replay                  import TrajPlayer
from teleoperation.robot_data_manager           import RobotDataManager
from teleoperation.interfaces.logging_config    import logger
from teleoperation.data_utils.robot_data        import *


POSE_STATES = ("pose_nav", "pose_hori", "pose_vert")
TELEOP_STATES = ("teleop", "teleop_l", "teleop_r")
THREADED_STATES = (*POSE_STATES, "replay")


@dataclass
class ActiveTraj:
    name: str
    qpos: List[np.ndarray]            # sequence of WB qpos frames
    idx_f: float = -1.0               # floating progress index
    rate: float = 1.0                 # playback speed
    N: int = 0                        # cached len(qpos)

    @classmethod
    def from_frames(cls, name: str, frames: List[np.ndarray], rate: float = 1.0):
        return cls(name=name, qpos=frames, rate=float(rate), N=len(frames))


def log_state_change(action: str, state_name: str):
    logger.info(f"{action:<6} {state_name:<12} state")


class RobotStateMachine:
    STATES = ['idle', 'navigation', 'pose_nav', 'pose_hori', 'pose_vert',
              'teleop', 'teleop_l', 'teleop_r', 'replay', 'fail', 'auto_task']

    def __init__(self, robot_data: RobotDataManager, p2p_planner: p2pPlanner, traj_player: TrajPlayer):
        self.robot_data  = robot_data
        self.p2p_planner = p2p_planner
        self.traj_player = traj_player

        self.teleop_solver = []
        self.init_cmd_once_state = False
        self.active_traj: Optional[ActiveTraj] = None

        self.th_is_running = threading.Event()
        self.threads: Dict[str, threading.Thread] = {}
        self.finished_flags: Dict[str, bool] = {}

        self.state_map = {
            'idle':         0,
            'fail':         1,
            'pose_nav':     2,
            'pose_hori':    3,
            'teleop':       4,
            'navigation':   5,
            'pose_vert':    6,
            'teleop_l':     7,
            'teleop_r':     8,
            'replay':       9,
            'auto_task':    10,
        }
        self._init_state_machine()

    # ---------------------------- FSM setup ----------------------------
    def _init_state_machine(self):
        transitions = [
            {'trigger': 'reset',                 'source': '*',       'dest': 'idle'},
            {'trigger': 'failed',                'source': '*',       'dest': 'fail'},
            {'trigger': 'th_start_nav',          'source': 'idle',    'dest': 'navigation'},
            {'trigger': 'th_reset_to_pose_nav',  'source': '*',       'dest': 'pose_nav'},
            {'trigger': 'th_reset_to_pose_hori', 'source': '*',       'dest': 'pose_hori'},
            {'trigger': 'th_reset_to_pose_vert', 'source': '*',       'dest': 'pose_vert'},
            {'trigger': 'th_start_teleop',       'source': 'idle',    'dest': 'teleop'},
            {'trigger': 'th_start_teleop_l',     'source': 'idle',    'dest': 'teleop_l'},
            {'trigger': 'th_start_teleop_r',     'source': 'idle',    'dest': 'teleop_r'},
            {'trigger': 'th_start_replay',       'source': 'idle',    'dest': 'replay'},
            {'trigger': 'th_start_auto_task',    'source': 'idle',    'dest': 'auto_task'},
        ]
        self.gm = GraphMachine(model=self, initial='idle', transitions=transitions, states=self.STATES)

    # ---------------------------- Robot data helpers ----------------------------
    def get_robot_qpos(self):
        return self.robot_data.get_robot_qpos()

    def get_latest_robot_wb_cmd(self):
        return self.robot_data.get_latest_robot_wb_cmd()

    def set_robot_wb_cmd_cache(self, robot_wb_cmd):
        self.robot_data.set_robot_wb_cmd_cache(robot_wb_cmd)

    def get_state_id(self) -> int:
        state_id = self.state_map.get(self.state, -1)
        self.robot_data.update_state_id(state_id)
        return state_id

    def in_idle(self) -> bool:
        return self.state == 'idle'

    def in_auto_task(self) -> bool:
        return self.state == 'auto_task'

    # ---------------------------- Thread helpers ----------------------------
    def _start_thread(self, name: str, target):
        log_state_change('enter', name)
        self.th_is_running.set()
        self.finished_flags[name] = False
        th = threading.Thread(target=target, daemon=True)
        self.threads[name] = th
        th.start()

    def _stop_thread(self, name: str):
        self.th_is_running.clear()
        th = self.threads.get(name)
        if th and not self.finished_flags.get(name, True):
            th.join(timeout=1.0)
            if th.is_alive():
                logger.warning(f"Thread {name} did not terminate cleanly.")
        log_state_change('exit', name)

    # ---------------------------- State hooks ----------------------------
    def on_enter_idle(self):
        log_state_change('enter', 'idle')
        if self.init_cmd_once_state:
            robot_wb_cmd = self.get_latest_robot_wb_cmd()
            robot_wb_cmd[S_CHASSIS] = np.zeros(DOF_CHASSIS, dtype=np.float64)
            self.set_robot_wb_cmd_cache(robot_wb_cmd)
        self.th_is_running.clear()

    def on_enter_fail(self):
        logger.info("Entered fail state due to exception.")
        self.th_is_running.clear()

    def on_enter_navigation(self): log_state_change('enter', 'navigation')
    def on_exit_navigation(self):  log_state_change('exit',  'navigation')

    # Teleop family – dedup logs
    def _sync_teleop_head_torso(self):
        robot_qpos = self.get_robot_qpos()
        robot_wb_cmd = self.get_latest_robot_wb_cmd()
        robot_wb_cmd[S_HEAD] = robot_qpos[S_HEAD]
        robot_wb_cmd[S_TORSO] = robot_qpos[S_TORSO]
        self.set_robot_wb_cmd_cache(robot_wb_cmd)

    def on_enter_teleop(self):
        self._sync_teleop_head_torso()
        log_state_change('enter', 'teleop_dual')
    def on_exit_teleop(self):    log_state_change('exit',  'teleop_dual')
    def on_enter_teleop_l(self):
        self._sync_teleop_head_torso()
        log_state_change('enter', 'teleop_l')
    def on_exit_teleop_l(self):  log_state_change('exit',  'teleop_l')
    def on_enter_teleop_r(self):
        self._sync_teleop_head_torso()
        log_state_change('enter', 'teleop_r')
    def on_exit_teleop_r(self):  log_state_change('exit',  'teleop_r')

    # Posed/replay states – unified threading pattern
    def on_enter_pose_nav(self):  self._start_thread('pose_nav',  lambda: self._motion_planner('pose_nav'))
    def on_exit_pose_nav(self):   self._stop_thread('pose_nav')
    def on_enter_pose_hori(self): self._start_thread('pose_hori', lambda: self._motion_planner('pose_hori'))
    def on_exit_pose_hori(self):  self._stop_thread('pose_hori')
    def on_enter_pose_vert(self): self._start_thread('pose_vert', lambda: self._motion_planner('pose_vert'))
    def on_exit_pose_vert(self):  self._stop_thread('pose_vert')
    def on_enter_replay(self):    self._start_thread('replay',    lambda: self._motion_planner('replay'))
    def on_exit_replay(self):     self._stop_thread('replay')

    def on_enter_auto_task(self): log_state_change('enter', 'auto_task')
    def on_exit_auto_task(self):  log_state_change('exit',  'auto_task')

    # ---------------------------- Planning & playback ----------------------------
    def _motion_planner(self, name: str):
        try:
            torso_mode = robot_const_proxy.TorsoConfig.CONTROL_MODE
            if name == 'replay':
                cfg = self.robot_data.teleop_data.get_task_params()
                speed_factor  = cfg.get('speed_factor', 1.0)
                force_closing = cfg.get('force_closing', [0, 0])
                traj_name     = cfg.get('traj_name', None)
                motion_traj = self.traj_player.planning(self.get_robot_qpos(), force_closing, traj_name, torso_mode)
            else:
                cfg = self.robot_data.teleop_data.get_task_params()
                head_mode  = cfg.get('head_mode', 'stay_still')
                motion_traj = self.p2p_planner.planning(self.get_robot_qpos(), name, 'keep', head_mode, torso_mode)
                speed_factor = 1.0

            # frames = [frame[S_WB_QPOS].copy() for frame in motion_traj]
            frames = motion_traj
            self.active_traj = ActiveTraj.from_frames(name, frames, speed_factor)
            self.finished_flags[name] = False
            self.th_is_running.set()

        except Exception as e:
            logger.error(e)
            self.failed()

    def step_traj(self):
        traj = self.active_traj
        if not traj or traj.N == 0:
            self.active_traj = None
            return None

        traj.idx_f = min(traj.idx_f + traj.rate, traj.N - 1)

        i0 = int(np.floor(traj.idx_f))
        i1 = min(i0 + 1, traj.N - 1)
        alpha = float(traj.idx_f - i0)

        q_interp = (1.0 - alpha) * traj.qpos[i0] + alpha * traj.qpos[i1]

        robot_wb_cmd = self.get_latest_robot_wb_cmd()
        robot_wb_cmd[S_WB_CMD] = q_interp
        robot_wb_cmd[S_CHASSIS] = np.zeros(DOF_CHASSIS, dtype=np.float64)

        if traj.idx_f >= traj.N - 1:
            self.finished_flags[traj.name] = True
            self.active_traj = None
            self.reset()
            self.th_is_running.clear()

        return robot_wb_cmd
