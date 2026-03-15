#!/usr/bin/env python3
"""测试 4: 零力矩模式（自由拖动）- 使用 FACTRTeleopFrankaMuJoCo + MuJoCo"""
import os
import sys
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_hardware.common import (
    separator, JOINT_NAMES, NUM_MOTORS,
    create_teleop, safe_shutdown, wait_for_enter, get_key_nonblocking,
)
import numpy as np


def main():
    teleop = create_teleop()
    if teleop is None:
        return
    driver = teleop.driver

    try:
        separator("测试 4: 零力矩模式")
        print("将启用力矩模式并发送零电流。")
        print("此模式下 Leader 臂应该可以自由手动拖动，但会有轻微阻力。")
        wait_for_enter("确认 Leader 臂处于安全位置后按 Enter 开始...")

        driver.set_torque_mode(False)
        driver.set_operating_mode(0)
        driver.set_torque_mode(True)
        print("[OK] 已切换到电流控制模式，力矩已启用")

        zero_current = [0.0] * NUM_MOTORS
        print("发送零电流，手动拖动各关节确认自由度正常。按 [q] 结束。\n")

        header = "  ".join([f"{n:>8s}" for n in JOINT_NAMES])
        print(f"{'':>6s}  {header}")
        print("-" * 80)

        while True:
            if hasattr(teleop, "viewer") and not teleop.viewer.is_running():
                print("\n\n[OK] MuJoCo 窗口已关闭，结束测试")
                break
            driver.set_current(zero_current)
            pos, vel = driver.get_positions_and_velocities()
            pos_deg = np.degrees(pos)
            pos_str = "  ".join([f"{p:+8.2f}" for p in pos_deg])
            print(f"  pos°: {pos_str}", end="\r")
            teleop.update_communication(
                (pos[:7] - teleop.joint_offsets[:7]) * teleop.joint_signs[:7],
                (pos[7] - teleop.joint_offsets[7]) * teleop.joint_signs[7],
            )
            if get_key_nonblocking() == 'q':
                print("\n\n[OK] 零力矩测试结束")
                break
            time.sleep(0.002)

        driver.set_current(zero_current)
    finally:
        print("\n正在安全关闭...")
        safe_shutdown(teleop)

    os._exit(0)


if __name__ == "__main__":
    main()
