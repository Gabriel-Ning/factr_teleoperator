# FACTR Dynamixel 遥操作

基于 Dynamixel 舵机的低成本力反馈遥操作，配合 MuJoCo 仿真。

基于 [FACTR](https://github.com/jasonjyliu/factr_teleop)（Jason Jingzhou Liu、Yulong Li）修改与扩展。

## 快速开始

**克隆本仓库时**请带上 submodule（MuJoCo 模型在 [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)）：

```bash
git clone --recurse-submodules <本仓库地址>
# 若已克隆未带 submodule，执行：
# git submodule update --init --recursive
```

```bash
# 安装依赖
uv sync

# 运行 MuJoCo 遥操作
uv run python main.py

# 运行硬件测试（示例：连接测试）
uv run python -m test_hardware.test_01_connection
```

## 串口设置（Dynamixel USB）

未配置时打开 USB 串口会报 **Permission denied**。需做一次性配置：

```bash
chmod +x scripts/setup_serial.sh
./scripts/setup_serial.sh
```

脚本会安装 udev 规则（`scripts/99-ftdi-dynamixel.rules`），为 FTDI 设备设置 `GROUP="dialout"`、`MODE="0660"`，并把当前用户加入 `dialout` 组；**脚本里已设置 USB 延迟（latency_timer）**，无需再手动改。

**重要：执行完脚本之后，必须 登出再登录（或重启），并 重新插拔 USB 线**，串口权限和延迟设置才会生效。

- 若使用其他 FTDI 设备，先改 `scripts/99-ftdi-dynamixel.rules` 里的 `ATTRS{serial}=="FTAO51S3"` 再执行脚本。
- 配置 YAML 可继续使用 `/dev/serial/by-id/usb-...` 路径。

## 项目结构

```
Factr_Dynamiexel/
├── main.py                 # 入口
├── pyproject.toml
├── src/
│   ├── factr_teleop/       # 遥操作核心
│   │   ├── configs/        # YAML 配置
│   │   ├── dynamixel/      # Dynamixel 驱动
│   │   ├── factr_teleop.py
│   │   └── factr_teleop_franka_mujoco.py
│   └── python_utils/       # 工具（ZMQ、配置等）
├── scripts/
│   ├── setup_serial.sh     # 串口/udev 一次性配置
│   └── 99-ftdi-dynamixel.rules
├── test_hardware/          # 硬件测试脚本
└── external/
    └── mujoco_menagerie/   # MuJoCo 机器人模型（git submodule，来自 google-deepmind/mujoco_menagerie）
```

更新 submodule：`git submodule update --remote external/mujoco_menagerie`
