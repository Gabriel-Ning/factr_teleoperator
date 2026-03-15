"""
test_hardware 公共模块 - 基于 factr_teleop.FACTRTeleopFrankaMuJoCo
所有测试使用 FACTRTeleopFrankaMuJoCo（含 MuJoCo viewer），复用 driver/calibration 逻辑。
"""

import os
import sys
import select
import termios
import tty
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(_THIS_DIR)

DEFAULT_CONFIG = os.path.join(
    PROJECT_ROOT, "src", "factr_teleop", "configs", "franka_mujoco.yaml"
)
CALIBRATION_OFFSETS_PATH = os.path.join(PROJECT_ROOT, "calibration_offsets.json")
URDF_PATH = os.path.join(
    PROJECT_ROOT, "src", "factr_teleop", "urdf", "factr_teleop_franka.urdf"
)


def _get_port_from_config():
    import yaml

    with open(DEFAULT_CONFIG) as f:
        cfg = yaml.safe_load(f)
    return "/dev/serial/by-id/" + cfg["dynamixel"]["dynamixel_port"]


# 兼容旧接口的常量（从 config 读取，此处为默认值）
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "Gripper"]
NUM_MOTORS = 8
NUM_ARM_JOINTS = 7
JOINT_SIGNS = np.array([1, 1, 1, -1, 1, -1, 1, -1], dtype=float)


def separator(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def wait_for_enter(msg="按 Enter 继续..."):
    input(f"\n{msg}")


def get_key_nonblocking():
    """非阻塞读取单键"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return None


def create_teleop(config_path=None, force_recalibrate=False, skip_match=True):
    """
    创建 FACTRTeleopFrankaMuJoCo（含 MuJoCo viewer），供 test 使用。
    返回 teleop 或 None（端口不存在时）。
    """
    config_path = config_path or DEFAULT_CONFIG
    try:
        from factr_teleop.factr_teleop_franka_mujoco import FACTRTeleopFrankaMuJoCo

        return FACTRTeleopFrankaMuJoCo(
            config_path=config_path,
            skip_match=skip_match,
            force_recalibrate=force_recalibrate,
        )
    except RuntimeError as e:
        if "driver" in str(e).lower() or "port" in str(e).lower():
            print(f"[FAIL] {e}")
            return None
        raise


def safe_shutdown(teleop_or_driver):
    """安全关闭：接受 teleop 或 driver"""
    try:
        if hasattr(teleop_or_driver, "shut_down"):
            teleop_or_driver.shut_down()
        elif hasattr(teleop_or_driver, "set_torque"):
            teleop_or_driver.set_torque(np.zeros(NUM_MOTORS))
            teleop_or_driver.set_torque_mode(False)
            teleop_or_driver.close()
        print("[OK] 舵机已关闭，端口已释放")
    except Exception:
        pass


def load_pinocchio_model():
    """加载 Pinocchio 模型（用于 test_08 重力补偿）"""
    import pinocchio as pin

    urdf_path = URDF_PATH
    urdf_dir = os.path.dirname(urdf_path)
    pin_model, _, _ = pin.buildModelsFromUrdf(filename=urdf_path, package_dirs=urdf_dir)
    pin_data = pin_model.createData()
    print(f"[OK] Pinocchio 模型已加载: {pin_model.nq} DoF")
    return pin_model, pin_data
