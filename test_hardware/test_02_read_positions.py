#!/usr/bin/env python3
"""测试 2: 持续读取关节位置 - 使用 FACTRTeleopFrankaMuJoCo + MuJoCo"""
import os
import sys
import time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_hardware.common import (
    separator, JOINT_NAMES, create_teleop, safe_shutdown, get_key_nonblocking,
)


def main():
    teleop = create_teleop()
    if teleop is None:
        return

    try:
        separator("测试 2: 读取关节位置和速度")
        print("手动转动 Leader 臂，MuJoCo 中可见 Follower 跟随。")
        print("按 [q] 结束此测试。\n")

        header = "  ".join([f"{n:>8s}" for n in JOINT_NAMES])
        print(f"{'':>6s}  {header}")
        print("-" * 80)

        while True:
            # 用户关闭 MuJoCo 窗口时也退出
            if hasattr(teleop, "viewer") and not teleop.viewer.is_running():
                print("\n\n[OK] MuJoCo 窗口已关闭，结束测试")
                break
            pos, vel, _, _ = teleop.get_leader_joint_states()
            pos_full = np.append(pos, teleop.gripper_pos)
            pos_deg = np.degrees(pos_full)
            pos_str = "  ".join([f"{p:+8.2f}" for p in pos_deg])
            print(f"  pos°: {pos_str}", end="\r")
            teleop.update_communication(pos, teleop.gripper_pos)
            if get_key_nonblocking() == 'q':
                print("\n\n[OK] 位置读取测试结束")
                break
            time.sleep(0.02)
    finally:
        print("\n正在安全关闭...")
        safe_shutdown(teleop)

    # GLFW/MuJoCo 线程会阻止进程退出
    os._exit(0)


if __name__ == "__main__":
    main()
