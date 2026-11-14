#!/usr/bin/env python3
"""手部骨骼可视化：收 UDP → 正向运动学 → 画骨架。

不需要 VaM，也不需要头显。用来确认手指弯曲方向、幅度、联动比例对不对。

用法：
    python3 tools/eic_hand_view.py --live              # 实时窗口
    python3 tools/eic_hand_view.py --save hand.png     # 出一张静态图
    python3 tools/eic_hand_view.py --save hand.png --from-sim   # 用合成数据

坐标系与固件一致：x+ 朝肘部，y+ 朝右，z+ 朝上。手指指向 -x。
"""

import argparse
import math
import queue
import socket
import struct
import sys
import threading

import numpy as np

UDP_PORT = 9001
PACKET_SIZE = 272

JOINT_NAMES = [
    "hand",
    "thumb1", "thumb2", "thumb3",
    "index1", "index2", "index3",
    "middle1", "middle2", "middle3",
    "ring1", "ring2", "ring3",
    "pinky1", "pinky2", "pinky3",
]

# 各指根在手掌上的位置（毫米），机体系
# 手掌长约 90mm 沿 -x，手指沿 y 排开
MCP = {
    "thumb":  (-28.0,  34.0, 0.0),
    "index":  (-85.0,  22.0, 0.0),
    "middle": (-90.0,   0.0, 0.0),
    "ring":   (-85.0, -22.0, 0.0),
    "pinky":  (-74.0, -42.0, 0.0),
}

# 三节骨骼长度（毫米），成人平均值
BONES = {
    "thumb":  (32.0, 25.0, 20.0),
    "index":  (40.0, 25.0, 20.0),
    "middle": (45.0, 28.0, 22.0),
    "ring":   (42.0, 26.0, 21.0),
    "pinky":  (33.0, 20.0, 17.0),
}

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]

# 画图用的颜色
COLORS = {
    "thumb":  "#d62728",
    "index":  "#1f77b4",
    "middle": "#2ca02c",
    "ring":   "#9467bd",
    "pinky":  "#ff7f0e",
}


def quat_to_matrix(q):
    """四元数 (w,x,y,z) 转 3x3 旋转矩阵"""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_identity():
    return (1.0, 0.0, 0.0, 0.0)


def parse_packet(data):
    """272 字节 → 16 个四元数，顺序见 JOINT_NAMES"""
    if len(data) != PACKET_SIZE or data[0:4] != b"EIGV":
        return None
    out = {}
    for i, name in enumerate(JOINT_NAMES):
        o = 16 + i * 16
        w, x, y, z = struct.unpack_from("<4f", data, o)
        out[name] = (w, x, y, z)
    return out


def solve(joints):
    """正向运动学，返回每根手指的关节世界坐标列表（含起点）"""
    q_hand = joints["hand"]
    R_hand = quat_to_matrix(q_hand)
    origin = np.zeros(3)

    chains = {}
    for finger in FINGER_ORDER:
        p = origin + R_hand @ np.array(MCP[finger])
        r = q_hand
        pts = [p.copy()]
        for i, length in enumerate(BONES[finger]):
            r = quat_mul(r, joints[f"{finger}{i + 1}"])
            R = quat_to_matrix(r)
            # 骨骼静止方向沿 -x
            d = R @ np.array([-length, 0.0, 0.0])
            p = p + d
            pts.append(p.copy())
        chains[finger] = np.array(pts)
    return chains


def draw(ax, chains, fps_text=""):
    ax.clear()
    # 手腕到各指根
    for finger in FINGER_ORDER:
        pts = chains[finger]
        ax.plot([0.0, pts[0][0]], [0.0, pts[0][1]], [0.0, pts[0][2]],
                color="#999999", lw=2.0, alpha=0.8)

    for finger in FINGER_ORDER:
        pts = chains[finger]
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                color=COLORS[finger], lw=3.0, marker="o", markersize=3)

    # 手腕
    ax.scatter([0.0], [0.0], [0.0], c="k", s=40, marker="s")

    ax.set_xlabel("X (toward elbow)")
    ax.set_ylabel("Y (right)")
    ax.set_zlabel("Z (up)")
    ax.set_xlim(-190, 40)
    ax.set_ylim(-80, 80)
    ax.set_zlim(-80, 80)
    ax.set_box_aspect((230, 160, 160))
    try:
        ax.set_proj_type("ortho")
    except AttributeError:
        pass
    if fps_text:
        ax.set_title(fps_text)


def run_live(port):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    q = queue.Queue(maxsize=2)

    def rx():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", port))
        while True:
            data, _ = s.recvfrom(2048)
            j = parse_packet(data)
            if j is None:
                continue
            if q.full():
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
            q.put(j)

    threading.Thread(target=rx, daemon=True).start()

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    state = {"joints": None, "count": 0}

    def update(_):
        try:
            state["joints"] = q.get_nowait()
            state["count"] += 1
        except queue.Empty:
            pass
        if state["joints"] is None:
            return
        chains = solve(state["joints"])
        draw(ax, chains, f"收到 {state['count']} 帧")
        ax.view_init(elev=25, azim=-60)

    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.show()
    return anim


def run_save(path, joints, from_sim):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if from_sim:
        sys.path.insert(0, __file__.rsplit("/", 2)[0])
        from tools.eic_bridge import frame_to_joints, iter_sim

        gen = iter_sim()
        # 取一个手指弯曲明显的时刻
        for k in range(60):
            frame = next(gen)
            j = frame_to_joints(frame)
        joints = {name: j[i] for i, name in enumerate(JOINT_NAMES)}

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    draw(ax, solve(joints), "hand skeleton")
    ax.view_init(elev=25, azim=-60)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"已保存 {path}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true", help="实时窗口")
    ap.add_argument("--save", help="保存一张静态图")
    ap.add_argument("--from-sim", action="store_true", help="静态图用合成数据")
    ap.add_argument("--port", type=int, default=UDP_PORT)
    args = ap.parse_args()

    if args.save:
        fake = None
        if not args.from_sim:
            # 没有指定数据源时给一个伸直手的默认姿态
            fake = {n: quat_identity() for n in JOINT_NAMES}
        return run_save(args.save, fake, args.from_sim)

    if args.live:
        run_live(args.port)
        return 0

    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
