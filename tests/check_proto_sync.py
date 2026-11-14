#!/usr/bin/env python3
"""检查驱动内嵌的协议解析与独立版本是否一致。

VaM 的脚本只能单文件，所以驱动里内嵌了一份 EIProtocol；Linux 上测试用的是
vam-plugin/EIGloveProtocol.cs。两份必须同步，否则测试通过但插件跑不对。

    python3 tests/check_proto_sync.py
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DRIVER = os.path.join(ROOT, "vam-plugin/EIGloveDriver.cs")
STANDALONE = os.path.join(ROOT, "vam-plugin/EIGloveProtocol.cs")

KEYS = [
    ("包长", r"PacketSize\s*=\s*(\d+)"),
    ("关节数", r"JointCount\s*=\s*(\d+)"),
    ("版本判断", r"version\s*!=\s*(\d+)"),
    ("头部偏移", r"16\s*\+\s*i\s*\*\s*16"),
    ("w 偏移", r"BitConverter\.ToSingle\(data,\s*o\)"),
    ("x 偏移", r"BitConverter\.ToSingle\(data,\s*o\s*\+\s*(\d+)\)"),
]


def extract(path):
    with open(path, encoding="utf-8") as f:
        s = f.read()
    found = {}
    for name, pat in KEYS:
        m = re.search(pat, s)
        found[name] = m.group(1) if m and m.groups() else ("hit" if m else None)
    return found


def main():
    a = extract(DRIVER)
    b = extract(STANDALONE)

    fails = 0
    print("== 协议实现同步检查 ==\n")
    for name, _ in KEYS:
        ok = a[name] == b[name] and a[name] is not None
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: 驱动={a[name]} 独立={b[name]}")
        if not ok:
            fails += 1

    print()
    if fails:
        print(f"有 {fails} 项不一致，改协议时两处都要动")
        return 1
    print("两份实现一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
