import numpy as np
from teleoperation.robot_config.robot_loader import robot_const_proxy
from teleoperation.data_utils.robot_data import *


def min_jerk_interpolation(jt_start, jt_goal, interp_speed=0.5, samples_per_second=50):
    jt_start = np.asarray(jt_start)
    jt_goal  = np.asarray(jt_goal)
    jt_delta = jt_goal - jt_start

    # Left arm interpolation time
    home_time_04_l = np.max(np.abs(jt_delta[S_ARM_L][:-3])) / interp_speed
    home_time_57_l = np.max(np.abs(jt_delta[S_ARM_L][-3:])) / (3 * interp_speed)
    home_time_l = max(home_time_04_l, home_time_57_l, 0.5)

    # Check if right arm joints exist
    if jt_start.size > 8:
        home_time_04_r = np.max(np.abs(jt_delta[S_ARM_R][:-3])) / interp_speed
        home_time_57_r = np.max(np.abs(jt_delta[S_ARM_R][-3:])) / (3 * interp_speed)
        home_time_r = max(home_time_04_r, home_time_57_r, 0.5)
        home_time = max(home_time_l, home_time_r)
    else:
        home_time = home_time_l

    steps = max(int(home_time * samples_per_second), 1)
    t_array = np.linspace(0, 1, steps, endpoint=False)
    alpha_array = 10 * t_array**3 - 15 * t_array**4 + 6 * t_array**5

    traj = jt_start + alpha_array[:, np.newaxis] * jt_delta
    traj = np.vstack([traj, jt_goal])
    return traj

class p2pPlanner(object):
    def __init__(self, config) -> None:
        '''
        load the offlline traj, and send them on demands
        '''
        self.config = config
        self.POSE_HORI  = np.array(robot_const_proxy.ArmConfig.TELEOP_READY_H_L   + robot_const_proxy.ArmConfig.TELEOP_READY_H_R)
        self.POSE_VERT  = np.array(robot_const_proxy.ArmConfig.TELEOP_READY_V_L   + robot_const_proxy.ArmConfig.TELEOP_READY_V_R)
        self.POSE_NAV   = np.array(robot_const_proxy.ArmConfig.WALK_READY_L       + robot_const_proxy.ArmConfig.WALK_READY_R)

    # =================================================================================(traj related)=================================================================================
    def planning(self, current_jt_config, goal_state, hand_mode, head_mode, torso_mode):
        head_mode = "keep"
        goal_jt_config = np.zeros(WB_QPOS_LEN, dtype=np.float64)
        goal_arm_configs = {
            'pose_hori':    self.POSE_HORI,
            'pose_vert':    self.POSE_VERT,
            'pose_nav':     self.POSE_NAV,
        }

        if goal_state not in goal_arm_configs:
            raise ValueError(f'Unknown goal state: {goal_state}')
        goal_jt_config[S_ARM] = goal_arm_configs[goal_state]

        if hand_mode=='keep':
            goal_jt_config[S_GRIPPER] = current_jt_config[S_GRIPPER]
            goal_jt_config[S_HAND]    = current_jt_config[S_HAND]

        if torso_mode=='pos':
            goal_jt_config[S_TORSO] = current_jt_config[S_TORSO]
        elif torso_mode=='vel':
            current_jt_config[S_TORSO] = 0.0

        p2p_motion = min_jerk_interpolation(current_jt_config, goal_jt_config, interp_speed=1.0)
        
        if head_mode=='lower_down':
            p2p_motion[:, S_HEAD] = [0.0, robot_const_proxy.HeadConfig.LOWER_DOWN]
        elif head_mode=="keep":
            p2p_motion[:, S_HEAD] = current_jt_config[S_HEAD]
        else:
            p2p_motion[:, S_HEAD] = np.zeros(DOF_HEAD)
        return p2p_motion

