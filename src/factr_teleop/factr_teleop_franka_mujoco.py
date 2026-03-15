# ---------------------------------------------------------------------------
# MuJoCo FR3 follower for FACTR teleoperation
# Based on FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# ---------------------------------------------------------------------------

import os
import logging
import numpy as np
import mujoco
import mujoco.viewer

from factr_teleop.factr_teleop import FACTRTeleop

logger = logging.getLogger(__name__)


class FACTRTeleopFrankaMuJoCo(FACTRTeleop):
    """
    Implements FACTR teleoperation with a MuJoCo FR3 simulation as the follower arm.

    The physical Dynamixel leader arm reads joint positions and sends them as position
    targets to a simulated Franka FR3 in MuJoCo. Contact forces from the simulation
    are fed back to the leader arm for force feedback.

    skip_match=True: 跳过初始位姿匹配（供 test_hardware 使用）
    force_recalibrate=True: 强制现场标定，不加载 calibration_offsets.json
    """

    def __init__(
        self,
        config_path: str,
        skip_match: bool = False,
        force_recalibrate: bool = False,
    ):
        self._skip_match = skip_match
        self._force_recalibrate = force_recalibrate
        super().__init__(config_path)

    def _match_start_pos(self):
        """子类覆盖：skip_match 时跳过"""
        if self._skip_match:
            logger.info(f"FACTR TELEOP {self.name}: Skipping start position match")
            return
        super()._match_start_pos()

    def _get_dynamixel_offsets(self, verbose=True):
        """子类覆盖：优先从 config dynamixel.joint_offsets 加载；force_recalibrate 时用 base 相同网格算法，步长 π/16"""
        if not self._force_recalibrate:
            stored = self.config.get("dynamixel", {}).get("joint_offsets", [])
            if stored and len(stored) == self.num_motors:
                self.joint_offsets = np.array(stored, dtype=float)
                if verbose:
                    logger.info("Loaded calibration offsets from config")
                return
        # 现场标定：与 base 相同算法，步长 π/16（base 为 π/2 或 π/8）
        for _ in range(10):
            self.driver.get_positions_and_velocities()

        def _get_error(calibration_joint_pos, offset, index, joint_state):
            joint_sign_i = self.joint_signs[index]
            joint_i = joint_sign_i * (joint_state[index] - offset)
            start_i = calibration_joint_pos[index]
            return np.abs(joint_i - start_i)

        self.joint_offsets = []
        curr_joints, _ = self.driver.get_positions_and_velocities()
        for i in range(self.num_arm_joints):
            best_offset = 0
            best_error = 1e9
            for offset in np.linspace(
                -20 * np.pi, 20 * np.pi, 20 * 32 + 1
            ):  # step π/16
                error = _get_error(self.calibration_joint_pos, offset, i, curr_joints)
                if error < best_error:
                    best_error = error
                    best_offset = offset
            self.joint_offsets.append(best_offset)

        curr_gripper_joint = curr_joints[-1]
        self.joint_offsets.append(curr_gripper_joint)
        self.joint_offsets = np.asarray(self.joint_offsets)
        if verbose:
            print(self.joint_offsets)
            print(
                "best offsets               : ",
                [f"{x:.3f}" for x in self.joint_offsets],
            )
            print(
                "best offsets function of pi: ["
                + ", ".join(
                    [
                        f"{int(np.round(x/(np.pi/2)))}*np.pi/2"
                        for x in self.joint_offsets
                    ]
                )
                + " ]"
            )

    def set_up_communication(self):
        # src/factr_teleop/ -> src/ -> project_root
        project_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        )
        mujoco_model_path = os.path.join(
            project_root,
            "external",
            self.config.get("mujoco", {}).get(
                "model_path", "mujoco_menagerie/franka_fr3/scene.xml"
            ),
        )
        logger.info(f"Loading MuJoCo model from: {mujoco_model_path}")

        self.mj_model = mujoco.MjModel.from_xml_path(mujoco_model_path)
        self.mj_data = mujoco.MjData(self.mj_model)

        self._joint_actuator_ids = []
        for i in range(1, 8):
            name = f"fr3_joint{i}"
            act_id = mujoco.mj_name2id(
                self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            if act_id == -1:
                raise RuntimeError(f"Actuator '{name}' not found in MuJoCo model")
            self._joint_actuator_ids.append(act_id)

        self._joint_qpos_ids = []
        for i in range(1, 8):
            name = f"fr3_joint{i}"
            jnt_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jnt_id == -1:
                raise RuntimeError(f"Joint '{name}' not found in MuJoCo model")
            qpos_adr = self.mj_model.jnt_qposadr[jnt_id]
            self._joint_qpos_ids.append(qpos_adr)

        home_pos = np.array(
            self.config["arm_teleop"]["initialization"]["initial_match_joint_pos"]
        )
        for idx, act_id in enumerate(self._joint_actuator_ids):
            self.mj_data.ctrl[act_id] = home_pos[idx]
        for idx, qpos_id in enumerate(self._joint_qpos_ids):
            self.mj_data.qpos[qpos_id] = home_pos[idx]
        mujoco.mj_forward(self.mj_model, self.mj_data)

        sim_steps = self.config.get("mujoco", {}).get("sim_steps_per_control", 1)
        self._sim_steps = sim_steps

        self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        logger.info("MuJoCo viewer launched.")

        self._external_torque = np.zeros(self.num_arm_joints)

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        for idx, act_id in enumerate(self._joint_actuator_ids):
            self.mj_data.ctrl[act_id] = leader_arm_pos[idx]

        for _ in range(self._sim_steps):
            mujoco.mj_step(self.mj_model, self.mj_data)

        self._external_torque = self.mj_data.qfrc_constraint[
            : self.num_arm_joints
        ].copy()

        if self.viewer.is_running():
            self.viewer.sync()

    def friction_compensation(self, arm_joint_vel):
        """Disabled: friction compensation is not needed for MuJoCo simulation."""
        return np.zeros(self.num_arm_joints)

    def get_leader_arm_external_joint_torque(self):
        return self._external_torque

    def get_leader_gripper_feedback(self):
        return 0.0

    def gripper_feedback(
        self, leader_gripper_pos, leader_gripper_vel, gripper_feedback
    ):
        return 0.0

    def shut_down(self):
        super().shut_down()
        if hasattr(self, "viewer") and self.viewer.is_running():
            self.viewer.close()
            logger.info("MuJoCo viewer closed.")
