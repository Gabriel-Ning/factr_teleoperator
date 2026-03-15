# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li
# ---------------------------------------------------------------------------

"""
FACTR 遥操作 + 真实 Franka：通过 ZMQ 与 Follower 通信，无 ROS 依赖。
与 base FACTRTeleop(config_path) 一致。
"""

import time
import logging
import numpy as np

from factr_teleop.factr_teleop import FACTRTeleop
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from python_utils.global_configs import (
    franka_left_real_zmq_addresses,
    franka_right_real_zmq_addresses,
)

logger = logging.getLogger(__name__)


class FACTRTeleopFrankaZMQ(FACTRTeleop):
    """
    通过 ZMQ 与真实 Franka Follower 通信：发送关节目标、接收关节状态与外力矩。
    无 ROS，仅 ZMQ。
    """

    def __init__(self, config_path: str):
        super().__init__(config_path)
        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"]["gain"]
        self.gripper_torque_ema_beta = self.config["controller"]["gripper_feedback"]["ema_beta"]
        self.gripper_external_torque = 0.0

    def set_up_communication(self):
        if self.name == "left":
            zmq_addresses = franka_left_real_zmq_addresses
        elif self.name == "right":
            zmq_addresses = franka_right_real_zmq_addresses
        else:
            raise ValueError(f"Invalid robot name '{self.name}'. Expected 'left' or 'right'.")

        self.franka_cmd_pub = ZMQPublisher(zmq_addresses["joint_pos_cmd_pub"])
        self.franka_joint_state_sub = ZMQSubscriber(zmq_addresses["joint_state_sub"])

        if self.enable_torque_feedback:
            self.franka_torque_sub = ZMQSubscriber(zmq_addresses["joint_torque_sub"])
            while self.franka_torque_sub.message is None:
                logger.info(f"Waiting for Franka {self.name} external joint torques...")
                time.sleep(0.1)

    def get_leader_gripper_feedback(self):
        return self.gripper_external_torque

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        torque_gripper = -1.0 * gripper_feedback / self.gripper_feedback_gain
        return torque_gripper

    def get_leader_arm_external_joint_torque(self):
        msg = self.franka_torque_sub.message
        if msg is None:
            return np.zeros(self.num_arm_joints)
        return np.asarray(msg, dtype=np.float64)[: self.num_arm_joints]

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        self.franka_cmd_pub.send_message(leader_arm_pos)
