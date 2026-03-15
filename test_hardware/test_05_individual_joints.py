#!/usr/bin/env python3
"""测试 5: 逐关节微力矩测试 - 使用 FACTRTeleopFrankaMuJoCo + MuJoCo"""
import os
import sys
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_hardware.common import (
    separator, JOINT_NAMES, NUM_MOTORS, JOINT_SIGNS,
    create_teleop, safe_shutdown, wait_for_enter,
)
import numpy as np


def main():
    teleop = create_teleop()
    if teleop is None:
        return
    driver = teleop.driver

    try:
        separator("测试 5: 逐关节微力矩测试")
        print("对每个关节施加一个很小的力矩脉冲，确认方向和响应。")
        wait_for_enter("确认 Leader 臂处于安全位置后按 Enter 开始...")

        driver.set_torque_mode(False)
        driver.set_operating_mode(0)
        driver.set_torque_mode(True)

        test_torque = 0.02
        for joint_idx in range(NUM_MOTORS):
            name = JOINT_NAMES[joint_idx]
            signed_torque = test_torque * JOINT_SIGNS[joint_idx]

            pos_before, _ = driver.get_positions_and_velocities()
            torques = np.zeros(NUM_MOTORS)
            torques[joint_idx] = signed_torque
            driver.set_torque(torques)
            time.sleep(0.3)

            pos_after, _ = driver.get_positions_and_velocities()
            driver.set_torque(np.zeros(NUM_MOTORS))
            time.sleep(0.1)

            delta_deg = np.degrees(pos_after[joint_idx] - pos_before[joint_idx])
            moved = abs(delta_deg) > 0.5
            status = "[OK]  " if moved else "[WARN]"
            print(f"  {status} {name} (ID {joint_idx+1}): delta = {delta_deg:+.2f}°  (torque = {signed_torque:+.4f} Nm)")

        driver.set_torque(np.zeros(NUM_MOTORS))
        print("\n所有关节测试完成。")
    finally:
        print("\n正在安全关闭...")
        safe_shutdown(teleop)

    os._exit(0)


if __name__ == "__main__":
    main()
