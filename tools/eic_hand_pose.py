#!/usr/bin/env python3
"""手部姿态控制器：拖拽 2D 骨骼模型，按 EIGV 协议把 16 关节四元数发给 VaM 插件。

用途：不戴手套、不接串口时手动摆姿势，验证 VaM 端骨骼映射与朝向补偿。

用法：
    python tools/eic_hand_pose.py                 # 默认 127.0.0.1:9001
    python tools/eic_hand_pose.py --udp-port 9001 --hand 1

交互：
    鼠标左键拖指尖 -> 弯曲该手指
    R -> 全部复位
    Esc -> 退出

协议见 docs/vam-协议.md：4 字节 "EIGV" + <HBBII> + 16 × 4 float，共 272 字节。
"""

import argparse
import math
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import font as tkfont

MAGIC = b"EIGV"
VERSION = 1
SEND_HZ = 200.0

# 关节下标，和 docs/vam-协议.md / vam-plugin 里 JOINT_NAMES 一致
JOINT_NAMES = [
    "hand",
    "thumb1", "thumb2", "thumb3",
    "index1", "index2", "index3",
    "middle1", "middle2", "middle3",
    "ring1", "ring2", "ring3",
    "pinky1", "pinky2", "pinky3",
]

# UI 里五根手指的顺序：拇指到小指。每指三节的关节下标起点
FINGER_JOINT_BASE = [1, 4, 7, 10, 13]   # thumb, index, middle, ring, pinky
FINGER_NAMES = ["拇指", "食指", "中指", "无名指", "小指"]

# 每节弯曲上限（度，相对上一节）。拖到 100% 时整根手指卷成拳
SEG_BEND = [45.0, 75.0, 55.0]

# 2D 布局：指根位置、初始方向（-90 表示向上）、三节长度（像素）
GEO = [
    ((285.0, 490.0), (-105.0), (62.0, 50.0, 40.0)),   # 拇指
    ((330.0, 425.0), (-90.0), (70.0, 55.0, 45.0)),    # 食指
    ((395.0, 415.0), (-90.0), (76.0, 58.0, 46.0)),    # 中指
    ((460.0, 425.0), (-90.0), (70.0, 55.0, 45.0)),    # 无名指
    ((525.0, 440.0), (-90.0), (60.0, 48.0, 40.0)),    # 小指
]

BG = "#1e1e28"
PALM_FILL = "#3a3a4a"
PALM_LINE = "#5a5a70"
BONE = "#e8e8f0"
JOINT = "#8ec5ff"
TIP = "#ffd34d"
TIP_DRAG = "#ff7a45"
TEXT = "#c8c8d8"
DIM = "#8a8aa0"


def rot_neg_y(deg: float):
    """绕 -y 轴旋转 deg 度的四元数 (w, x, y, z)。和 eic_bridge 的 sim 一致。"""
    h = math.radians(deg) * 0.5
    return (math.cos(h), 0.0, -math.sin(h), 0.0)


class PoseApp:
    def __init__(self, root: tk.Tk, host: str, port: int, hand: int):
        self.root = root
        self.host = host
        self.port = port
        self.hand = hand

        self.bend = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.drag = None
        self.running = True
        self.sent = 0
        self._last_count = 0
        self._last_rate_time = time.time()
        self._rate = 0.0

        root.title("EIGlove 手部姿态控制器")
        root.configure(bg=BG)
        root.resizable(False, False)

        self.font = tkfont.Font(family="Microsoft YaHei UI", size=11)
        self.small = tkfont.Font(family="Microsoft YaHei UI", size=9)

        self.canvas = tk.Canvas(root, width=760, height=620, bg=BG, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Button-1>", self.on_down)
        self.canvas.bind("<B1-Motion>", self.on_motion)
        self.canvas.bind("<ButtonRelease-1>", self.on_up)
        root.bind("<Key-r>", lambda e: self.reset())
        root.bind("<Key-R>", lambda e: self.reset())
        root.bind("<Escape>", lambda e: self.quit())

        self.redraw()
        self.tick()

        self.thread = threading.Thread(target=self.send_loop, daemon=True)
        self.thread.start()

    # ---------- 几何 ----------

    def fk(self, finger: int, c: float):
        """返回 [指根, 近节末端, 中节末端, 指尖] 四个 2D 点。"""
        base, base_deg, lengths = GEO[finger]
        pts = [base]
        acc = 0.0
        for seg in range(3):
            acc += SEG_BEND[seg] * c
            ang = math.radians(base_deg + acc)
            x = pts[-1][0] + lengths[seg] * math.cos(ang)
            y = pts[-1][1] + lengths[seg] * math.sin(ang)
            pts.append((x, y))
        return pts

    def bend_for_point(self, finger: int, mx: float, my: float) -> float:
        """网格搜索让指尖最接近鼠标的弯度。"""
        best_c, best_d = 0.0, float("inf")
        for k in range(101):
            c = k / 100.0
            p = self.fk(finger, c)[-1]
            d = (p[0] - mx) ** 2 + (p[1] - my) ** 2
            if d < best_d:
                best_c, best_d = c, d
        return best_c

    # ---------- 交互 ----------

    def on_down(self, ev):
        best, best_d = None, 40.0 * 40.0
        for i in range(5):
            tip = self.fk(i, self.bend[i])[-1]
            d = (tip[0] - ev.x) ** 2 + (tip[1] - ev.y) ** 2
            if d < best_d:
                best, best_d = i, d
        if best is not None:
            self.drag = best
            self.redraw()

    def on_motion(self, ev):
        if self.drag is None:
            return
        self.bend[self.drag] = self.bend_for_point(self.drag, ev.x, ev.y)
        self.redraw()

    def on_up(self, ev):
        self.drag = None
        self.redraw()

    def reset(self):
        self.bend = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.redraw()

    # ---------- 绘制 ----------

    def redraw(self):
        cv = self.canvas
        cv.delete("all")

        # 手掌
        cv.create_polygon(
            250, 425, 560, 425, 560, 575, 250, 575,
            fill=PALM_FILL, outline=PALM_LINE, width=3,
        )
        cv.create_text(405, 555, text="手掌（掌心朝右）", fill=DIM, font=self.small)

        for i in range(5):
            pts = self.fk(i, self.bend[i])
            dragging = (self.drag == i)
            for k in range(3):
                cv.create_line(
                    pts[k][0], pts[k][1], pts[k + 1][0], pts[k + 1][1],
                    fill=BONE, width=8, capstyle="round",
                )
            for k in range(1, 3):
                cv.create_oval(
                    pts[k][0] - 8, pts[k][1] - 8, pts[k][0] + 8, pts[k][1] + 8,
                    fill=JOINT, outline="",
                )
            # 指尖：可拖
            tip = pts[3]
            r = 14 if dragging else 12
            cv.create_oval(
                tip[0] - r, tip[1] - r, tip[0] + r, tip[1] + r,
                fill=TIP_DRAG if dragging else TIP, outline="#7a5a00", width=2,
            )
            label = "%s %3d%%" % (FINGER_NAMES[i], int(round(self.bend[i] * 100)))
            cv.create_text(tip[0] + 22, tip[1] - 14, text=label, fill=TEXT,
                           font=self.font, anchor="w")

        # 顶部/底部状态
        cv.create_text(
            20, 22,
            text="UDP %s:%d  |  已发 %d 包  |  %d Hz  |  拖指尖弯曲，R 复位，Esc 退出"
                 % (self.host, self.port, self.sent, int(self._rate)),
            fill=TEXT, font=self.font, anchor="w",
        )

    # ---------- 发送 ----------

    def build_joints(self):
        out = [(1.0, 0.0, 0.0, 0.0)] * 16
        for i in range(5):
            c = self.bend[i]
            for seg in range(3):
                out[FINGER_JOINT_BASE[i] + seg] = rot_neg_y(SEG_BEND[seg] * c)
        return out

    def send_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        seq = 0
        period = 1.0 / SEND_HZ
        while self.running:
            t0 = time.perf_counter()
            joints = self.build_joints()
            buf = bytearray()
            buf += MAGIC
            buf += struct.pack(
                "<HBBII", VERSION, self.hand, 0,
                seq & 0xFFFFFFFF,
                int(time.time() * 1000) & 0xFFFFFFFF,
            )
            for q in joints:
                buf += struct.pack("<4f", *q)
            try:
                sock.sendto(bytes(buf), (self.host, self.port))
                self.sent += 1
            except OSError:
                pass
            seq += 1
            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)

    def tick(self):
        now = time.time()
        if now - self._last_rate_time >= 0.5:
            self._rate = (self.sent - self._last_count) / (now - self._last_rate_time)
            self._last_count = self.sent
            self._last_rate_time = now
        self.redraw()
        self.root.after(33, self.tick)

    def quit(self):
        self.running = False
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=9001)
    ap.add_argument("--hand", type=int, default=1, choices=[0, 1])
    args = ap.parse_args()

    root = tk.Tk()
    PoseApp(root, args.host, args.udp_port, args.hand)
    root.mainloop()


if __name__ == "__main__":
    main()
