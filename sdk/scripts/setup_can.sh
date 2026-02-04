#!/bin/bash
# LinkerHand CAN 接口配置脚本
#
# 用法:
#   ./setup_can.sh [接口名称] [波特率]
#
# 示例:
#   ./setup_can.sh can0 1000000
#   ./setup_can.sh can1 500000

set -e

INTERFACE="${1:-can0}"
BITRATE="${2:-1000000}"

echo "========================================"
echo "LinkerHand CAN 接口配置"
echo "========================================"
echo "接口: $INTERFACE"
echo "波特率: $BITRATE"
echo "----------------------------------------"

# 检查是否有 root 权限
if [ "$EUID" -ne 0 ]; then
  echo "需要 root 权限，使用 sudo 重新运行..."
  exec sudo "$0" "$@"
fi

# 先关闭接口（如果已存在）
echo "关闭现有接口..."
ip link set $INTERFACE down 2>/dev/null || true

# 设置波特率
echo "设置波特率为 $BITRATE..."
ip link set $INTERFACE type can bitrate $BITRATE

# 启用接口
echo "启用接口..."
ip link set $INTERFACE up

# 显示状态
echo "----------------------------------------"
echo "配置完成! 接口状态:"
echo ""
ip -details link show $INTERFACE
echo ""
echo "========================================"
echo "现在可以运行测试程序:"
echo "  ./build/test_force_sensor -i $INTERFACE"
echo "========================================"
