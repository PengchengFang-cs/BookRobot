import numpy as np
from robot_utils.math.pose_utils import *
from robot_utils.math.transformations import euler_from_quaternion, quaternion_dispQ, quaternion_from_matrix, pose_quat_2_mat

from teleoperation.interfaces.logging_config    import logger
from teleoperation.data_utils.robot_data        import S_ARM_L, S_ARM_R, S_GRIPPER, S_HEAD, S_TORSO, S_CHASSIS
from teleoperation.controllers.base_controller  import BaseController
from teleoperation.robot_config.robot_loader    import robot_const_proxy


class HeadController(BaseController):
    def _init_mems(self):
        self.init_trans_mat_head = []
        self.init_trans_quat_head = {}

    def init_solver(self, arm_selection):
        self.left_arm_enabled, self.right_arm_enabled = arm_selection
        self._store_vr_init_pose()

    def _store_vr_init_pose(self):
        if self.robot_data_manager.teleop_data.get_task_params().get('head_mode', 'stay_still') == 'follow_head':
            for axis in ['head_z', 'head_y']:
                mat, quat, _ = self.robot_data_manager.get_vr_pose_mat_and_quat(axis)
                self.init_trans_quat_head[axis] = quat
                self.init_trans_mat_head = mat  # overwritten for each axis

    def update(self):
        mode = self.robot_data_manager.teleop_data.get_task_params().get('head_mode', 'stay_still')
        mode = 'stay_still'
        mode_methods = {
            'stay_still':   self._mode_stay_still,
            'lower_down':   self._mode_lower_down,
            'follow_head':  self._mode_follow_head,
            'follow_hand':  self._mode_follow_hand,
        }
        head_cmd_rad = mode_methods.get(mode, self._mode_stay_still)()
        self.robot_data_manager.current_robot_state.update_cache(head_cmd_jt_pos=head_cmd_rad)

    def _mode_stay_still(self):
        return self.robot_data_manager.current_robot_state.get_latest_wb_cmd_head_jt_pos()

    def _mode_lower_down(self):
        return [0.0, robot_const_proxy.HeadConfig.LOWER_DOWN]

    def _mode_follow_head(self):
        head_rad = self.robot_data_manager.teleop_data.get_cmd_params().get('head',[0.0, 0.0])
        head_rad_mod = head_rad.copy()
        head_rad_mod[1] = head_rad_mod[1] * 2
        filtered = self.robot_data_manager.filter_bank.cmd_smoothing(head_rad_mod, 'head')
        return np.clip(filtered, robot_const_proxy.HeadConfig.HEAD_MIN, robot_const_proxy.HeadConfig.HEAD_MAX)

    def _mode_follow_hand(self):
        head_cmd_rad = self.compute_head_by_hand()
        return self.robot_data_manager.filter_bank.cmd_smoothing(head_cmd_rad, 'head_auto')

    def compute_head_by_hand(self):
        # compute hand speed
        v_left,v_right = self._get_ee_pose_vel_norm()
        if abs(v_left) > 0.10 and abs(v_right) > 0.10:
            v_left  = 0.15
            v_right = 0.15
        else:
            v_left  = 0 if abs(v_left)  < 0.13 else v_left
            v_right = 0 if abs(v_right) < 0.13 else v_right

        v_sum = v_left * int(self.left_arm_enabled) + v_right * int(self.right_arm_enabled)
        if v_sum==0:
            return self.robot_data_manager.current_robot_state.get_latest_wb_cmd_head_jt_pos()
        else:
            w_left  = v_left * int(self.left_arm_enabled)  / v_sum
            w_right = v_right * int(self.right_arm_enabled) / v_sum
            attention_pt = w_left * self.robot_data_manager.current_robot_state.get_latest_wb_cmd_ee_pose('left') + w_right * self.robot_data_manager.current_robot_state.get_latest_wb_cmd_ee_pose('right')

            yaw   = np.clip(0.6 * np.arctan2(attention_pt[1], attention_pt[0]), robot_const_proxy.HeadConfig.HEAD_MIN[0], robot_const_proxy.HeadConfig.HEAD_MAX[0])
            pitch = np.clip(0.25 * np.arctan2(0.2-attention_pt[2], attention_pt[0]), robot_const_proxy.HeadConfig.HEAD_MIN[1], robot_const_proxy.HeadConfig.HEAD_MAX[1])
            return np.array([yaw, pitch])

    def get_head_angle(self):
        head_rad = np.zeros(2)
        _, _, z = euler_from_quaternion(quaternion_dispQ(self.init_trans_quat_head['head_z'][3:], self.robot_data_manager.filter_bank.cmd_smoothing(self.robot_data_manager.get_vr_pose_mat_and_quat('head_z')[1][3:], 'head_frame_z')))
        head_rad[0] = z
        _, y, _ = euler_from_quaternion(quaternion_dispQ(self.init_trans_quat_head['head_y'][3:], self.robot_data_manager.filter_bank.cmd_smoothing(self.robot_data_manager.get_vr_pose_mat_and_quat('head_y')[1][3:], 'head_frame_y')))
        head_rad[1] = -y
        return head_rad

    def _get_ee_pose_vel_norm(self):
        history = self.robot_data_manager.current_robot_state.get_ee_pose_cmd_history(min_len=11)
        if history is None:
            return np.zeros(2, dtype=float)

        t1, t2 = history['timestamps'][1], history['timestamps'][-1]
        pose1_l, pose2_l = history['left'][1], history['left'][-1]
        pose1_r, pose2_r = history['right'][1], history['right'][-1]

        dt = t2 - t1
        if dt <= 0:
            return np.zeros(2, dtype=float)

        vel_l = np.linalg.norm(pose2_l[:3] - pose1_l[:3]) / dt
        vel_r = np.linalg.norm(pose2_r[:3] - pose1_r[:3]) / dt
        v_left  = self.robot_data_manager.filter_bank.cmd_smoothing(vel_l, 'hand_goal_l')
        v_right = self.robot_data_manager.filter_bank.cmd_smoothing(vel_r, 'hand_goal_r')
        return np.array([v_left, v_right], dtype=float)

