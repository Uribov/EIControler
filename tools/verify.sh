#!/bin/bash
# 一键跑完所有不依赖硬件的验证。
# 用法：bash tools/verify.sh

set -e
cd "$(dirname "$0")/.."

echo "===== 1. 编译固件 ====="
pio run 2>&1 | tail -4

echo
echo "===== 2. 姿态算法离线测试 ====="
gcc -O2 -Wall -Wextra -I src -o /tmp/ei_test_vqf tests/test_vqf.c src/lib/vqf.c -lm
/tmp/ei_test_vqf

echo
echo "===== 3. 帧协议 C 端打包 ====="
gcc -O2 -Wall -Wextra -I src -o /tmp/ei_test_frame tests/test_frame.c src/lib/frame_codec.c -lm
/tmp/ei_test_frame /tmp/ei_frame.bin

echo
echo "===== 4. Python 端解析对拍 ====="
python3 tools/eic_frame.py --dump /tmp/ei_frame.bin

echo
echo "===== 5. 协议自检 ====="
python3 tools/eic_frame.py

echo
echo "===== 6. 上位机到可视化全链路 ====="
python3 -u tests/test_hand_view.py

echo
echo "===== 全部验证通过 ====="
