#!/usr/bin/env python3
"""测试 1: 串口连接 & 舵机通信 - 使用 FACTRTeleopFrankaMuJoCo + MuJoCo"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_hardware.common import (
    separator, NUM_MOTORS, _get_port_from_config,
    create_teleop, safe_shutdown,
)


def main():
    port = _get_port_from_config()
    separator("测试 1: 串口连接 & 舵机通信")
    print(f"端口: {port}")
    print(f"电机数量: {NUM_MOTORS}")

    if not os.path.exists(port):
        print(f"\n[FAIL] 端口 {port} 不存在！")
        print("  请检查 USB 连接，运行 ls /dev/serial/by-id/ 确认。")
        return

    print(f"[OK] 端口存在")
    resolved = os.path.realpath(port)
    print(f"[OK] 解析为: {resolved}")

    ttydev = os.path.basename(resolved)
    latency_path = f"/sys/bus/usb-serial/devices/{ttydev}/latency_timer"
    try:
        with open(latency_path) as f:
            latency = int(f.read().strip())
        if latency == 1:
            print(f"[OK] {ttydev} latency_timer = {latency}")
        else:
            print(f"[WARN] {ttydev} latency_timer = {latency} (应为 1)")
            print(f"  运行: echo 1 | sudo tee {latency_path}")
    except Exception as e:
        print(f"[WARN] 无法读取 latency_timer: {e}")

    print(f"\n正在连接 {NUM_MOTORS} 个舵机 (ID 1-{NUM_MOTORS})...")
    teleop = create_teleop()
    if teleop is None:
        return

    print(f"[OK] 驱动初始化成功，所有 {NUM_MOTORS} 个舵机已连接")
    print("\n正在安全关闭...")
    safe_shutdown(teleop)
    print("\n[OK] 测试 1 完成")
    # MuJoCo/GLFW 线程会阻止进程退出，用 os._exit 立即终止（不跑 atexit/析构）
    os._exit(0)


if __name__ == "__main__":
    main()
