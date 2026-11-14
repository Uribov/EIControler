#!/usr/bin/env python3
"""链路自检：上位机发 UDP → 可视化端收包 → 正向运动学，验证整条通路。

不需要 GUI，也不需要板子。

    python3 tests/test_hand_view.py
"""

import math
import os
import socket
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from tools.eic_bridge import frame_to_joints, iter_sim  # noqa: E402
from tools.eic_hand_view import (  # noqa: E402
    BONES,
    JOINT_NAMES,
    MCP,
    parse_packet,
    quat_to_matrix,
    solve,
)

PORT = 9101
FAILS = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}")
    if not ok:
        FAILS.append(name)


def joint_angle(q):
    """四元数对应的旋转角，单位度"""
    w = max(-1.0, min(1.0, abs(q[0])))
    return math.degrees(2.0 * math.acos(w))


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("127.0.0.1", PORT))
    srv.settimeout(5.0)

    proc = subprocess.Popen(
        [sys.executable, "-u", os.path.join(ROOT, "tools/eic_bridge.py"),
         "--sim", "--udp-port", str(PORT)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    packets = []
    try:
        for _ in range(10):
            data, _ = srv.recvfrom(2048)
            packets.append(data)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
        srv.close()

    print(f"\n== UDP 包 ==")
    check("收到 10 包", len(packets) == 10, f"实际 {len(packets)}")
    check("包长 272", all(len(p) == 272 for p in packets))

    joints = parse_packet(packets[-1])
    check("能解析", joints is not None)
    check("关节数 16", len(joints) == 16, f"实际 {len(joints)}")

    print(f"\n== 正向运动学 ==")
    chains = solve(joints)
    check("五根手指都有解", len(chains) == 5, f"实际 {len(chains)}")

    # 每根手指必须有 4 个点（起点 + 三节）
    all_ok = all(len(c) == 4 for c in chains.values())
    check("每指 4 个关节", all_ok)

    # 骨架总长应与骨骼表吻合
    for finger in ("index", "middle", "pinky"):
        pts = chains[finger]
        total = sum(float(((pts[i + 1] - pts[i]) ** 2).sum()) ** 0.5 for i in range(3))
        want = sum(BONES[finger])
        check(f"{finger} 长度 {total:.1f}mm", abs(total - want) < 1.0, f"期望 {want}")

    print(f"\n== 弯曲方向 ==")
    # 合成数据每根手指绕 y 轴弯，弯曲后指尖应在掌心侧（z 为负）
    tipped = chains["middle"][-1]
    root = chains["middle"][0]
    check("中指指尖低于指根", tipped[2] < root[2],
          f"指尖 z={tipped[2]:.1f} 指根 z={root[2]:.1f}")

    print(f"\n== 角度统计 ==")
    for finger in ("index", "middle"):
        angs = [joint_angle(joints[f"{finger}{i + 1}"]) for i in range(3)]
        print(f"  {finger}: 近节 {angs[0]:5.1f}°  中节 {angs[1]:5.1f}°  远节 {angs[2]:5.1f}°")
        # 远节 = 中节 × 0.66
        if angs[1] > 2.0:
            ratio = angs[2] / angs[1]
            check(f"{finger} 远节联动比 {ratio:.2f}", abs(ratio - 0.66) < 0.05)

    print()
    if FAILS:
        print(f"有 {len(FAILS)} 项失败")
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
