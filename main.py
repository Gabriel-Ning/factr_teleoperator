#!/usr/bin/env python3
"""
FACTR Teleoperation - MuJoCo FR3 Follower
用 Leader Arm (Dynamixel) 控制 MuJoCo 中的 Franka FR3 仿真。
"""

import os
import logging
import argparse

from factr_teleop.factr_teleop_franka_mujoco import FACTRTeleopFrankaMuJoCo

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))


def main():
    parser = argparse.ArgumentParser(
        description="FACTR Teleop with MuJoCo FR3 follower"
    )
    parser.add_argument(
        "--config",
        type=str,
        default=os.path.join(
            PROJECT_ROOT, "src", "factr_teleop", "configs", "franka_mujoco.yaml"
        ),
        help="Path to the YAML config file",
    )
    parser.add_argument(
        "--skip-match",
        action="store_true",
        help="跳过位姿匹配，直接开始遥操作（默认需先将 Leader 摆到 initial_match_joint_pos 附近）",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    teleop = FACTRTeleopFrankaMuJoCo(
        config_path=args.config, skip_match=args.skip_match
    )
    teleop.run()


if __name__ == "__main__":
    main()
