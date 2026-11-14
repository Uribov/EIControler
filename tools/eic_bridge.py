#!/usr/bin/env python3
"""手套上位机：串口收帧 → 解算手部关节 → UDP 发给 VaM 插件。

用法：
    python3 tools/eic_bridge.py --port /dev/ttyACM0     # 真实串口
    python3 tools/eic_bridge.py --replay cap.bin        # 回放抓包
    python3 tools/eic_bridge.py --sim                   # 合成数据自测

协议见 docs/vam-协议.md。
"""

import argparse
import math
import socket
import struct
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])

from tools.eic_frame import FRAME_SIZE, parse_stream  # noqa: E402

UDP_HOST = "127.0.0.1"
UDP_PORT = 9001
MAGIC = b"EIGV"
VERSION = 1

# 关节下标，和 docs/vam-协议.md 的表一一对应
JOINT_NAMES = [
    "hand",
    "thumb1", "thumb2", "thumb3",
    "index1", "index2", "index3",
    "middle1", "middle2", "middle3",
    "ring1", "ring2", "ring3",
    "pinky1", "pinky2", "pinky3",
]
N_JOINTS = len(JOINT_NAMES)

# 右手版：CS1..CS10 依次是小指近/中、无名指近/中、中指近/中、食指近/中、拇指近/中
FINGER_MAP = [
    ("pinky", 0, 1),
    ("ring", 2, 3),
    ("middle", 4, 5),
    ("index", 6, 7),
    ("thumb", 8, 9),
]

DISTAL_RATIO = 0.66  # 远节相对中节的联动比例


# ---------- 四元数基础 ----------

def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_norm(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / n for c in q)


def q_slerp(a, b, t):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-c for c in b)
        dot = -dot
    if dot > 0.9995:
        return q_norm(tuple(x + t * (y - x) for x, y in zip(a, b)))
    th = math.acos(max(-1.0, min(1.0, dot)))
    s = math.sin(th)
    wa = math.sin((1 - t) * th) / s
    wb = math.sin(t * th) / s
    return q_norm(tuple(wa * x + wb * y for x, y in zip(a, b)))


Q_IDENT = (1.0, 0.0, 0.0, 0.0)


# ---------- 帧 → 关节 ----------

def frame_to_joints(frame: dict) -> list:
    """把一帧姿态转成 16 个关节的相对四元数。"""
    q_hand = tuple(frame["q_hand"])
    q_rel = [tuple(q) for q in frame["q_rel"]]

    out = [Q_IDENT] * N_JOINTS
    out[0] = q_norm(q_hand)

    for name, near_i, mid_i in FINGER_MAP:
        q_near = q_norm(q_rel[near_i])          # 近节相对手掌
        q_mid = q_norm(q_rel[mid_i])            # 中节相对手掌

        # 骨骼层级是 中节相对近节，所以要把公共的手掌系除掉
        q_mid_local = q_norm(q_mul(q_conj(q_near), q_mid))

        # 远节按联动比例缩放中节的旋转角
        q_distal = q_norm(q_slerp(Q_IDENT, q_mid_local, DISTAL_RATIO))

        idx = JOINT_NAMES.index(name + "1")
        out[idx] = q_near
        out[idx + 1] = q_mid_local
        out[idx + 2] = q_distal

    return out


def pack_packet(seq: int, ts_ms: int, hand: int, joints: list) -> bytes:
    buf = bytearray()
    buf += MAGIC
    buf += struct.pack("<HBBII", VERSION, hand, 0, seq & 0xFFFFFFFF, ts_ms & 0xFFFFFFFF)
    for q in joints:
        buf += struct.pack("<4f", *q)
    assert len(buf) == 272, f"包长 {len(buf)}"
    return bytes(buf)


# ---------- 数据源 ----------

def iter_replay(path: str):
    with open(path, "rb") as f:
        data = f.read()
    frames = parse_stream(data)
    for f in frames:
        yield f


def iter_serial(port: str, baud: int = 115200):
    import serial  # 延迟导入，没装也能跑 --sim

    ser = serial.Serial(port, baud, timeout=0.05)
    buf = bytearray()
    while True:
        chunk = ser.read(4096)
        if not chunk:
            continue
        buf += chunk
        # 保留最后的半帧，前面的交给流式解析
        keep = FRAME_SIZE - 1
        frames = parse_stream(bytes(buf))
        if len(buf) > keep * 8:
            buf = buf[-keep:]
        for f in frames:
            yield f


def iter_sim(rate_hz: float = 200.0):
    """合成数据：每根手指独立做正弦弯曲，用来验证链路与观感。"""
    t = 0.0
    dt = 1.0 / rate_hz
    while True:
        q_rel = []
        for i in range(10):
            finger = i // 2
            is_mid = i % 2
            amp = 0.25 + 0.15 * finger
            phase = t * (1.0 + 0.2 * finger) * 2 * math.pi
            angle = amp * (0.5 + 0.5 * math.sin(phase)) * (1.0 if is_mid else 0.6)
            # 绕 -y 轴弯曲，指尖朝掌心（机体系 -z）方向落下
            q = (math.cos(angle / 2), 0.0, -math.sin(angle / 2), 0.0)
            q_rel.append(list(q_norm(q)))
        frame = {
            "seq": 0,
            "ts_ms": int(t * 1000),
            "q_hand": [1.0, 0.0, 0.0, 0.0],
            "q_rel": q_rel,
            "temps": [30.0] * 12,
        }
        yield frame
        t += dt


# ---------- 主循环 ----------

def main() -> int:
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="串口设备，如 /dev/ttyACM0")
    src.add_argument("--replay", help="回放抓包文件")
    src.add_argument("--sim", action="store_true", help="合成数据")
    ap.add_argument("--host", default=UDP_HOST)
    ap.add_argument("--udp-port", type=int, default=UDP_PORT)
    ap.add_argument("--hand", type=int, default=0, choices=[0, 1])
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if args.replay:
        source = iter_replay(args.replay)
    elif args.sim:
        source = iter_sim()
    else:
        source = iter_serial(args.port)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.udp_port)

    seq = 0
    sent = 0
    t0 = time.monotonic()
    last_report = t0

    for frame in source:
        joints = frame_to_joints(frame)
        pkt = pack_packet(seq, int(time.monotonic() * 1000), args.hand, joints)
        sock.sendto(pkt, target)
        seq += 1
        sent += 1

        if args.verbose and sent % 40 == 0:
            n = JOINT_NAMES.index("index2")
            q = joints[n]
            ang = 2 * math.degrees(math.acos(max(-1.0, min(1.0, abs(q[0])))))
            print(f"seq={seq} index2 弯曲={ang:6.1f}°")

        now = time.monotonic()
        if now - last_report >= 2.0:
            fps = sent / (now - t0)
            print(f"[bridge] 已发 {sent} 包，平均 {fps:.1f} fps -> {args.host}:{args.udp_port}")
            last_report = now

        if args.sim:
            time.sleep(1.0 / 200.0)

    print(f"[bridge] 数据源结束，共发 {sent} 包")
    return 0


if __name__ == "__main__":
    sys.exit(main())
