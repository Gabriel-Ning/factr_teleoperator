#!/usr/bin/env python3
"""测试 9: Sys-ID 数据采集 - 使用 FACTRTeleopFrankaMuJoCo + MuJoCo"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import json
import pinocchio as pin
import numpy as np
from test_hardware.common import (
    separator, PROJECT_ROOT, NUM_ARM_JOINTS, JOINT_NAMES, URDF_PATH,
    create_teleop, safe_shutdown, wait_for_enter, get_key_nonblocking,
    load_pinocchio_model,
)


def main():
    separator("测试 9: 系统辨识数据采集")
    print("采集多组 (关节位置, 重力力矩) 数据用于惯性参数辨识。")
    print("  1. 手动将 Leader 臂拖到一个姿态")
    print("  2. 按 [空格] 采样当前位姿")
    print("  3. 重复多次，覆盖不同姿态 (建议 20+ 组)")
    print("  4. 按 [q] 或关闭 MuJoCo 窗口结束采集并保存")
    wait_for_enter("将 Leader 臂摆到校准位姿后按 Enter...")

    teleop = create_teleop()
    if teleop is None:
        return

    try:
        pin_model, pin_data = load_pinocchio_model()
        print("[OK] 校准 offset 已从 config 加载\n")

        teleop.driver.set_torque_mode(False)
        teleop.driver.set_operating_mode(0)
        teleop.driver.set_torque_mode(True)
        teleop.driver.set_current([0.0] * 8)

        samples = []
        print("准备就绪。自由拖动 Leader 臂到不同姿态，按 [空格] 采样，[q] 结束。\n")

        header = "  ".join([f"{'J'+str(i+1):>7s}" for i in range(NUM_ARM_JOINTS)])
        print(f"  {'#':>3s}  {header}  |  {header}")
        print(f"  {'':>3s}  {'--- 关节角度 (deg) ---':^56s}  |  {'--- URDF 重力力矩 (Nm) ---':^56s}")
        print("-" * 130)

        while True:
            if hasattr(teleop, "viewer") and not teleop.viewer.is_running():
                print("\n\n[OK] MuJoCo 窗口已关闭，结束采集")
                break

            arm_pos, arm_vel, _, _ = teleop.get_leader_joint_states()
            teleop.driver.set_current([0.0] * 8)
            teleop.update_communication(arm_pos, teleop.gripper_pos)

            key = get_key_nonblocking()
            if key == ' ':
                tau_g = pin.rnea(pin_model, pin_data, arm_pos, np.zeros_like(arm_vel), np.zeros_like(arm_vel))
                sample = {"joint_pos_rad": arm_pos.tolist(), "tau_gravity_nm": tau_g.tolist()}
                samples.append(sample)
                n = len(samples)
                pos_str = "  ".join([f"{np.degrees(p):+7.2f}" for p in arm_pos])
                tau_str = "  ".join([f"{t:+7.4f}" for t in tau_g])
                print(f"  {n:3d}  {pos_str}  |  {tau_str}")

            elif key == 'q':
                break

            pos_deg = np.degrees(arm_pos)
            live_str = "  ".join([f"{p:+7.2f}" for p in pos_deg])
            print(f"  ... {live_str}  (space=采样, q=结束)", end="\r")
            time.sleep(0.02)

        teleop.driver.set_current([0.0] * 8)

        if samples:
            out_path = os.path.join(PROJECT_ROOT, "sysid_data.json")
            output = {
                "urdf": os.path.basename(URDF_PATH),
                "num_samples": len(samples),
                "joint_names": JOINT_NAMES[:NUM_ARM_JOINTS],
                "samples": samples,
            }
            with open(out_path, "w") as f:
                json.dump(output, f, indent=2)
            print(f"\n\n[OK] 已采集 {len(samples)} 组数据，保存到 {out_path}")
            all_tau = np.array([s["tau_gravity_nm"] for s in samples])
            print(f"\n  各关节重力力矩范围 (Nm):")
            for i in range(NUM_ARM_JOINTS):
                t_min, t_max = all_tau[:, i].min(), all_tau[:, i].max()
                t_mean = all_tau[:, i].mean()
                print(f"    J{i+1}: min={t_min:+.4f}  max={t_max:+.4f}  mean={t_mean:+.4f}")
        else:
            print("\n\n[WARN] 未采集任何数据")
    finally:
        print("\n正在安全关闭...")
        safe_shutdown(teleop)

    os._exit(0)


if __name__ == "__main__":
    main()
