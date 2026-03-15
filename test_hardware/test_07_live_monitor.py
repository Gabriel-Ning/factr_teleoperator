#!/usr/bin/env python3
"""测试 7: 实时监控（校准后角度）- 使用 FACTRTeleopFrankaMuJoCo + MuJoCo"""
import os
import sys
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_hardware.common import (
    separator, JOINT_NAMES, create_teleop, safe_shutdown,
    wait_for_enter, get_key_nonblocking,
)
import numpy as np


def main():
    separator("测试 7: 实时监控（校准后坐标）")
    print("显示经过校准的关节角度，MuJoCo 中可见 Follower 跟随。")
    wait_for_enter("将 Leader 臂摆到校准位姿后按 Enter...")

    teleop = create_teleop()
    if teleop is None:
        return

    try:
        print("[OK] 校准完成，开始实时监控。移动 Leader 臂观察 MuJoCo 中变化。按 [q] 结束。\n")

        arm_names = "  ".join([f"{n:>8s}" for n in JOINT_NAMES[:7]])
        print(f"{'':>6s}  {arm_names}  {'Gripper':>8s}")
        print("-" * 80)

        while True:
            if hasattr(teleop, "viewer") and not teleop.viewer.is_running():
                print("\n\n[OK] MuJoCo 窗口已关闭，结束测试")
                break
            arm_pos, _, grip_pos, _ = teleop.get_leader_joint_states()
            cal_deg = np.degrees(arm_pos)
            grip_deg = np.degrees(grip_pos)
            pos_str = "  ".join([f"{p:+8.2f}" for p in cal_deg])
            print(f"  deg:  {pos_str}  {grip_deg:+8.2f}", end="\r")
            teleop.update_communication(arm_pos, grip_pos)
            if get_key_nonblocking() == 'q':
                print("\n\n[OK] 实时监控结束")
                break
            time.sleep(0.02)
    finally:
        print("\n正在安全关闭...")
        safe_shutdown(teleop)

    os._exit(0)


if __name__ == "__main__":
    main()
