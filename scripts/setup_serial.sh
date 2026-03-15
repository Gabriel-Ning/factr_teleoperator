#!/usr/bin/env bash
# 串口权限与 udev 设置，便于仓库复用时一键配置 Dynamixel 所用 USB 串口。
# 用法: ./scripts/setup_serial.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RULES_NAME="99-ftdi-dynamixel.rules"
RULES_SRC="$SCRIPT_DIR/$RULES_NAME"
RULES_DST="/etc/udev/rules.d/$RULES_NAME"

echo "=== FACTR Dynamixel 串口设置 ==="

# 1. 安装 udev 规则（需 sudo）
if [[ ! -f "$RULES_SRC" ]]; then
  echo "错误: 未找到 $RULES_SRC"
  exit 1
fi

echo "安装 udev 规则: $RULES_SRC -> $RULES_DST"
sudo cp "$RULES_SRC" "$RULES_DST"
sudo chmod 644 "$RULES_DST"

# 2. 重载 udev
echo "重载 udev 规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 3. 将当前用户加入 dialout，以便无需 root 打开串口
echo "将用户 $USER 加入组 dialout ..."
sudo usermod -aG dialout "$USER"

echo ""
echo "--- 完成 ---"
echo "请 重新登录 或 重启，使 dialout 组生效；然后重新插拔 USB 串口。"
echo "验证: ls -l /dev/serial/by-id/ 或 ls -l /dev/my_ftdi"
echo "若配置使用 by-id 路径，无需改 YAML；若使用固定名可改为 /dev/my_ftdi。"
