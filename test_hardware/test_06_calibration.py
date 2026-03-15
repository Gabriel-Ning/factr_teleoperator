#!/usr/bin/env python3
"""测试 6: 关节校准偏移量计算 - 使用 FACTRTeleopFrankaMuJoCo + MuJoCo"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedSeq
from test_hardware.common import (
    separator, DEFAULT_CONFIG,
    create_teleop, safe_shutdown, wait_for_enter,
)
import numpy as np


def main():
    separator("测试 6: 校准偏移量")
    print("将 Leader 臂手动摆到校准位姿:")
    print("  calibration_joint_pos = [0, -45°, 0, -135°, 0, 90°, 0]")
    print("  (对应弧度: [0, -0.7854, 0, -2.356, 0, 1.57, 0])")
    print("\n各关节允许 ±90° 误差，大致对准即可。")
    wait_for_enter("摆好后按 Enter 开始校准...")

    teleop = create_teleop(force_recalibrate=True)
    if teleop is None:
        return

    try:
        curr_joints, _ = teleop.driver.get_positions_and_velocities()
        print(f"\n原始舵机读数 (rad): {[f'{x:.4f}' for x in curr_joints]}")
        print(f"原始舵机读数 (deg): {[f'{np.degrees(x):.1f}' for x in curr_joints]}")

        joint_offsets = teleop.joint_offsets
        print(f"\n计算得到的偏移量:")
        print(f"  offsets (rad): {[f'{x:.4f}' for x in joint_offsets]}")
        print(f"  offsets (π/2): {[f'{int(np.round(x/(np.pi/2)))}*π/2' for x in joint_offsets]}")

        arm_pos, _, _, _ = teleop.get_leader_joint_states()
        cal_pos = teleop.calibration_joint_pos
        print(f"\n校准后关节位置 (rad): {[f'{x:.4f}' for x in arm_pos]}")
        print(f"校准后关节位置 (deg): {[f'{np.degrees(x):.1f}' for x in arm_pos]}")
        print(f"期望校准位置   (deg): {[f'{np.degrees(x):.1f}' for x in cal_pos]}")

        error = np.abs(arm_pos - cal_pos)
        max_err = np.max(error)
        max_err_deg = np.degrees(max_err)
        print(f"\n最大校准误差: {max_err_deg:.2f}°")
        if max_err < np.radians(10):
            print("[OK] 校准精度良好")
        elif max_err < np.radians(15):
            print("[OK] 校准可接受（误差 < 15°）")
        else:
            print("[WARN] 校准误差较大，请重新摆放 Leader 臂后再试")
        if max_err < np.radians(15):
            yaml = YAML()
            yaml.preserve_quotes = True
            with open(DEFAULT_CONFIG) as f:
                cfg = yaml.load(f)
            cfg.setdefault("dynamixel", {})
            offsets_list = CommentedSeq([round(float(x), 6) for x in joint_offsets])
            offsets_list.fa.set_flow_style()
            cfg["dynamixel"]["joint_offsets"] = offsets_list
            with open(DEFAULT_CONFIG, "w") as f:
                yaml.dump(cfg, f)
            print(f"\n[OK] offset 已写入配置 {DEFAULT_CONFIG}")
    finally:
        print("\n正在安全关闭...")
        safe_shutdown(teleop)

    os._exit(0)


if __name__ == "__main__":
    main()
