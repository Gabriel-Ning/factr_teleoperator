# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li
# ---------------------------------------------------------------------------

"""
重力补偿演示：仅 Leader 臂重力补偿 + 零空间调节，无 Follower 通信。
无 ROS 依赖，与 base FACTRTeleop(config_path) 一致。
"""

import logging
import numpy as np

from factr_teleop.factr_teleop import FACTRTeleop

logger = logging.getLogger(__name__)


class FACTRTeleopGravComp(FACTRTeleop):
    """
    仅演示 Leader 臂的重力补偿与零空间调节，不实现与 Follower 的通信。
    """

    def __init__(self, config_path: str):
        super().__init__(config_path)

    def set_up_communication(self):
        pass

    def get_leader_gripper_feedback(self):
        return 0.0

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        return 0.0

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        pass

    def get_leader_arm_external_joint_torque(self):
        return np.zeros(self.num_arm_joints)
