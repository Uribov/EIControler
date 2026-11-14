#!/usr/bin/env python3
"""EI Controller 姿态帧的打包与解析。

与固件 src/io/io.c 保持同一份协议：

    [2B 帧头 0xAA55] [2B 序号] [4B 时间戳 ms] [16B 手掌四元数]
    [10 × 16B 相对四元数] [12 × 2B 温度 0.01℃] [2B CRC16-CCITT]

整帧 210 字节，小端。

用法：
    python3 tools/eic_frame.py            # 自检
    python3 tools/eic_frame.py --dump f.bin
"""

import struct
import sys

FRAME_SIZE = 210
HDR = b"\xAA\x55"
N_FINGER = 10
N_IMU = 12


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def build_frame(seq: int, ts_ms: int, q_hand, q_rel, temps) -> bytes:
    """q_hand: 4 floats；q_rel: 10×4 floats；temps: 12 floats（摄氏度）"""
    assert len(q_hand) == 4
    assert len(q_rel) == N_FINGER and all(len(q) == 4 for q in q_rel)
    assert len(temps) == N_IMU

    body = bytearray()
    body += HDR
    body += struct.pack("<H", seq & 0xFFFF)
    body += struct.pack("<I", ts_ms & 0xFFFFFFFF)
    body += struct.pack("<4f", *q_hand)
    for q in q_rel:
        body += struct.pack("<4f", *q)
    for t in temps:
        raw = int(round(t * 100.0))
        raw = max(-32768, min(32767, raw))
        body += struct.pack("<h", raw)

    body += struct.pack("<H", crc16_ccitt(bytes(body)))
    assert len(body) == FRAME_SIZE, f"帧长 {len(body)} != {FRAME_SIZE}"
    return bytes(body)


def parse_frame(buf: bytes) -> dict:
    if len(buf) != FRAME_SIZE:
        raise ValueError(f"帧长不对：{len(buf)}")
    if buf[0:2] != HDR:
        raise ValueError("帧头不对")

    crc_rx = struct.unpack_from("<H", buf, FRAME_SIZE - 2)[0]
    crc_calc = crc16_ccitt(buf[: FRAME_SIZE - 2])
    if crc_rx != crc_calc:
        raise ValueError(f"CRC 不对：收到 0x{crc_rx:04X}，算出 0x{crc_calc:04X}")

    o = 2
    seq = struct.unpack_from("<H", buf, o)[0]
    o += 2
    ts_ms = struct.unpack_from("<I", buf, o)[0]
    o += 4

    q_hand = list(struct.unpack_from("<4f", buf, o))
    o += 16

    q_rel = []
    for _ in range(N_FINGER):
        q_rel.append(list(struct.unpack_from("<4f", buf, o)))
        o += 16

    temps = []
    for _ in range(N_IMU):
        temps.append(struct.unpack_from("<h", buf, o)[0] / 100.0)
        o += 2

    return {
        "seq": seq,
        "ts_ms": ts_ms,
        "q_hand": q_hand,
        "q_rel": q_rel,
        "temps": temps,
    }


def parse_stream(data: bytes):
    """从连续的字节流里找出所有合法帧。日志文本混在里面会被跳过。

    做法是扫帧头，命中后试解析，CRC 不过就往前挪一个字节继续找。
    """
    frames = []
    i = 0
    n = len(data)
    while i + FRAME_SIZE <= n:
        if data[i] == 0xAA and data[i + 1] == 0x55:
            try:
                frames.append(parse_frame(data[i : i + FRAME_SIZE]))
                i += FRAME_SIZE
                continue
            except ValueError:
                pass
        i += 1
    return frames


def _self_test() -> int:
    import math
    import random

    random.seed(7)

    q_hand = [1.0, 0.0, 0.0, 0.0]
    q_rel = []
    for _ in range(N_FINGER):
        a = random.uniform(-0.3, 0.3)
        b = random.uniform(-0.3, 0.3)
        n = math.sqrt(1 + a * a + b * b)
        q_rel.append([1 / n, a / n, b / n, 0.0])
    temps = [random.uniform(25.0, 35.0) for _ in range(N_IMU)]

    frame = build_frame(65533, 123456, q_hand, q_rel, temps)
    got = parse_frame(frame)

    ok = True
    if got["seq"] != 65533:
        print("序号不对")
        ok = False
    if got["ts_ms"] != 123456:
        print("时间戳不对")
        ok = False
    for a, b in zip(got["q_hand"], q_hand):
        if abs(a - b) > 1e-6:
            print("手掌四元数不对")
            ok = False
            break
    for i in range(N_FINGER):
        for a, b in zip(got["q_rel"][i], q_rel[i]):
            if abs(a - b) > 1e-6:
                print(f"相对四元数 {i} 不对")
                ok = False
                break
    for a, b in zip(got["temps"], temps):
        if abs(a - b) > 0.011:
            print("温度不对")
            ok = False
            break

    # CRC 必须能抓错
    bad = bytearray(frame)
    bad[40] ^= 0xFF
    try:
        parse_frame(bytes(bad))
        print("损坏帧没有被 CRC 拦住")
        ok = False
    except ValueError:
        pass

    # 流式解析必须能跳过日志文本
    noise = b"[0;32mI (1234) io: sent 100\r\n"
    stream = noise + frame + b"\x00\x11" + frame
    got_frames = parse_stream(stream)
    if len(got_frames) != 2:
        print(f"流式解析应得 2 帧，实际 {len(got_frames)}")
        ok = False

    print("帧自检：" + ("全部通过" if ok else "有失败"))
    print(f"  帧长 {len(frame)} 字节，CRC=0x{struct.unpack_from('<H', frame, 208)[0]:04X}")
    return 0 if ok else 1


def main() -> int:
    if len(sys.argv) >= 3 and sys.argv[1] == "--dump":
        with open(sys.argv[2], "rb") as f:
            data = f.read()
        for i in range(0, len(data), FRAME_SIZE):
            chunk = data[i : i + FRAME_SIZE]
            try:
                r = parse_frame(chunk)
            except ValueError as e:
                print(f"第 {i // FRAME_SIZE} 帧解析失败：{e}")
                continue
            q = r["q_hand"]
            print(f"seq={r['seq']:5d} ts={r['ts_ms']:10d} q_hand=({q[0]:+.3f} {q[1]:+.3f} {q[2]:+.3f} {q[3]:+.3f})")
        return 0

    return _self_test()


if __name__ == "__main__":
    sys.exit(main())
