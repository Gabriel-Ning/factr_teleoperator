#!/usr/bin/env python3
"""测试 3: 通信速率测试 - 使用 FACTRTeleopFrankaMuJoCo + MuJoCo"""
import os
import sys
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_hardware.common import separator, create_teleop, safe_shutdown


def main():
    teleop = create_teleop()
    if teleop is None:
        return
    driver = teleop.driver

    try:
        separator("测试 3: 通信速率测试")
        print("测量 1000 次读取的平均用时，验证能否达到 500Hz...")

        n_samples = 1000
        t_start = time.perf_counter()
        for _ in range(n_samples):
            driver.get_positions_and_velocities()
        elapsed = time.perf_counter() - t_start

        avg_ms = (elapsed / n_samples) * 1000
        rate_hz = n_samples / elapsed
        print(f"  总耗时: {elapsed:.3f}s")
        print(f"  单次读取: {avg_ms:.2f} ms")
        print(f"  通信速率: {rate_hz:.0f} Hz")

        if rate_hz >= 450:
            print(f"[OK] 速率 {rate_hz:.0f} Hz 满足 500Hz 控制要求")
        elif rate_hz >= 200:
            print(f"[WARN] 速率 {rate_hz:.0f} Hz 偏低，可运行但建议降低 controller.frequency")
        else:
            print(f"[FAIL] 速率 {rate_hz:.0f} Hz 过低！请检查 latency_timer 和波特率设置")
    finally:
        print("\n正在安全关闭...")
        safe_shutdown(teleop)

    os._exit(0)


if __name__ == "__main__":
    main()
