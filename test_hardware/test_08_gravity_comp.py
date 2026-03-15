#!/usr/bin/env python3
"""测试 8: 重力补偿调试 - 使用 FACTRTeleopFrankaMuJoCo + MuJoCo + tkinter 滑块"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import json
import pinocchio as pin
import numpy as np
import tkinter as tk
from tkinter import ttk

from test_hardware.common import (
    separator, PROJECT_ROOT, NUM_ARM_JOINTS,
    create_teleop, safe_shutdown, wait_for_enter, load_pinocchio_model,
)


def build_gain_panel(params, on_save):
    """创建 tkinter 控制面板，params 为可变 dict，滑块实时更新"""
    root = tk.Tk()
    root.title("重力补偿参数")
    root.geometry("320x220")
    root.resizable(False, False)

    main = ttk.Frame(root, padding=10)
    main.pack(fill=tk.BOTH, expand=True)

    # gravity_gain
    ttk.Label(main, text="gravity_gain (0~2):").pack(anchor=tk.W)
    g_var = tk.DoubleVar(value=params["gravity_gain"])
    g_scale = ttk.Scale(main, from_=0.0, to=2.0, variable=g_var, orient=tk.HORIZONTAL, length=260)
    g_scale.pack(fill=tk.X, pady=(0, 4))

    def update_gravity(v):
        val = float(v)
        params["gravity_gain"] = val
        g_label.config(text=f"  {val:.2f}")

    g_scale.configure(command=update_gravity)
    g_label = ttk.Label(main, text=f"  {params['gravity_gain']:.2f}")
    g_label.pack(anchor=tk.W)

    # stiction_gain
    ttk.Label(main, text="stiction_gain (0~1):").pack(anchor=tk.W, pady=(8, 0))
    s_var = tk.DoubleVar(value=params["stiction_gain"])
    s_scale = ttk.Scale(main, from_=0.0, to=1.0, variable=s_var, orient=tk.HORIZONTAL, length=260)
    s_scale.pack(fill=tk.X, pady=(0, 4))

    def update_stiction(*_):
        val = s_var.get()
        params["stiction_gain"] = val
        s_label.config(text=f"  {val:.2f}")

    s_scale.configure(command=update_stiction)
    s_label = ttk.Label(main, text=f"  {params['stiction_gain']:.2f}")
    s_label.pack(anchor=tk.W)

    # friction_enable
    f_var = tk.BooleanVar(value=params["friction_enable"])
    f_cb = ttk.Checkbutton(main, text="摩擦补偿 (friction_enable)", variable=f_var)
    f_cb.pack(anchor=tk.W, pady=(8, 0))

    def on_friction(*_):
        params["friction_enable"] = f_var.get()

    f_var.trace_add("write", on_friction)

    # 保存按钮
    def do_save():
        on_save(params)
        root.after(100, lambda: status.config(text="已保存"))

    btn = ttk.Button(main, text="保存参数", command=do_save)
    btn.pack(pady=(12, 4))
    status = ttk.Label(main, text="")
    status.pack(anchor=tk.W)

    def on_closing():
        params["_closed"] = True
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    return root, g_var, s_var, f_var


def main():
    print("测试 8 启动中...", flush=True)
    teleop = create_teleop()
    if teleop is None:
        print("[FAIL] 无法创建驱动，请检查串口连接", flush=True)
        return

    try:
        separator("测试 8: 重力补偿模式")
        print("基于 URDF 惯性参数，使用 Pinocchio RNEA 计算重力补偿力矩")
        print("并实时施加到舵机上。如果补偿正确，手松开后 Leader 臂应悬停不动。")
        print("")
        print("MuJoCo 窗口显示 Leader 臂位姿，右侧控制面板可调节 gain。")
        print("关闭 MuJoCo 窗口或控制面板即退出。")

        pin_model, pin_data = load_pinocchio_model()
        wait_for_enter("将 Leader 臂摆到校准位姿后按 Enter...")

        print("[OK] 校准 offset 已从 config 加载")

        # 加载/默认参数
        params_path = os.path.join(PROJECT_ROOT, "gravity_comp_params.json")
        if os.path.exists(params_path):
            with open(params_path) as f:
                saved = json.load(f)
            params = {
                "gravity_gain": saved.get("gravity_comp_gain", 0.85),
                "friction_enable": saved.get("friction_enable", True),
                "stiction_gain": saved.get("stiction_gain", 0.6),
            }
            print(f"[OK] 已加载参数: {params_path}")
        else:
            params = {"gravity_gain": 0.85, "friction_enable": True, "stiction_gain": 0.6}
        params["_closed"] = False

        def save_params(p):
            out = {
                "gravity_comp_gain": round(float(p["gravity_gain"]), 4),
                "friction_enable": p["friction_enable"],
                "stiction_gain": round(float(p["stiction_gain"]), 4),
            }
            with open(params_path, "w") as f:
                json.dump(out, f, indent=2)
            print(f"\n[OK] 参数已保存到 {params_path}\n")

        # 使用 teleop 自带的 MuJoCo viewer（create_teleop 已创建）
        viewer = teleop.viewer
        print("[OK] MuJoCo 窗口已打开")

        # tkinter 控制面板
        root, g_var, s_var, f_var = build_gain_panel(params, save_params)
        root.after(100, root.lift)

        stiction_speed = 0.9
        stiction_dither = np.ones(NUM_ARM_JOINTS, dtype=bool)
        initial_pos = None
        loop_count = 0

        print("\n开始重力补偿... (关闭 MuJoCo 或控制面板退出)\n")

        while viewer.is_running() and not params.get("_closed", False):
            t_start = time.perf_counter()
            arm_pos, arm_vel, grip_pos, _ = teleop.get_leader_joint_states()

            if initial_pos is None:
                initial_pos = arm_pos.copy()

            gravity_gain = params["gravity_gain"]
            friction_enable = params["friction_enable"]
            stiction_gain = params["stiction_gain"]

            tau_g = pin.rnea(pin_model, pin_data, arm_pos, arm_vel, np.zeros_like(arm_vel))
            tau_g_scaled = tau_g * gravity_gain

            tau_friction = np.zeros(NUM_ARM_JOINTS)
            if friction_enable:
                for i in range(NUM_ARM_JOINTS):
                    if abs(arm_vel[i]) < stiction_speed:
                        sign = 1.0 if stiction_dither[i] else -1.0
                        tau_friction[i] = sign * stiction_gain * abs(tau_g_scaled[i])
                        stiction_dither[i] = ~stiction_dither[i]

            torque_arm = tau_g_scaled + tau_friction
            teleop.set_leader_joint_torque(torque_arm, 0.0)

            # 同步 MuJoCo 显示
            teleop.update_communication(arm_pos, grip_pos)

            # 更新 tkinter
            try:
                root.update()
            except tk.TclError:
                break

            loop_count += 1
            if loop_count % 25 == 0:
                drift = np.degrees(np.linalg.norm(arm_pos - initial_pos))
                tau_str = "  ".join([f"{t:+6.4f}" for t in torque_arm])
                fric_mark = "F" if friction_enable else " "
                print(f"  g={gravity_gain:.2f} {fric_mark} {tau_str}  {drift:6.2f}°", end="\r")

            elapsed = time.perf_counter() - t_start
            sleep_time = 0.002 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        try:
            root.destroy()
        except tk.TclError:
            pass

        print("\n\n[OK] 重力补偿测试结束")
        print(f"\n最终参数: gravity_gain={params['gravity_gain']:.2f}, stiction_gain={params['stiction_gain']:.2f}")
    finally:
        print("\n正在安全关闭...")
        safe_shutdown(teleop)

    os._exit(0)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[ERROR] {e}", flush=True)
        import traceback
        traceback.print_exc()
        sys.exit(1)
