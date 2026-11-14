# 交接：Windows 上要做的事

这份是给 Windows 那边的 DSH 看的。项目在 Linux 上做到某个程度了，剩下的活只能在 Windows 干。

## 这是什么

体感手套主控固件 + VaM 接入。ESP32-S3 带 12 颗 IMU 和 1 颗磁力计，片上算姿态，USB 输出，上位机转成手部骨骼发给 VaM 插件。

```
手套 ──USB──> eic_bridge.py ──UDP 9001──> VaM 插件 ──> 人体骨骼
```

## 现在的状态

| 部分 | 状态 |
|---|---|
| 固件 | 全部模块写完，编译通过，Flash 9.6% |
| 姿态算法 | PC 端五个场景测试通过 |
| 帧协议 | C 与 Python 双向对拍通过 |
| 上位机 | 三种数据源都能跑，合成数据 193fps |
| 骨骼可视化 | 能画出 16 关节的手，弯曲方向验证过 |
| VaM 插件 | C# 写完，协议解析单测过，**没在 VaM 里编译过** |
| 硬件实测 | 完全没做，板子还没到 |

## Windows 第一次上手做什么

按这个顺序，每一层不通就停：

### 1. 确认 Python 能跑

```powershell
python tools\eic_bridge.py --sim --verbose
```

看到每秒发 200 包左右、打印手指弯曲角度就对了。这一步不需要 VaM，也不需要硬件。

### 2. 把插件放进 VaM

把 `vam-plugin\EIGloveDriver.cs` 复制到 VaM 目录下的 `Custom\Scripts\`。

打开 VaM，加载一个人物，选中人体 Atom → Add Plugin → 选 `EIGloveDriver`。

### 3. 看日志

VaM 的日志窗口会打印：

```
[EIGlove] 找到 N/16 个骨骼（右手）
[EIGlove] 初始化完成，监听 UDP 9001
```

**N 不是 16 就是骨骼名对不上。** 插件会把人物身上所有手部相关骨骼名完整列出来，照着改文件里的 `BoneNamesRight` 数组。

### 4. 通数据

VaM 里插件加载好之后，在另一个终端跑：

```powershell
python tools\eic_bridge.py --sim
```

虚拟手应该开始动。如果日志显示「还没收到数据」，检查：

- 上位机是否真的在发（`--verbose` 能看到）
- 端口是不是 9001
- Windows 防火墙有没有拦 Python 的 UDP

### 5. 调朝向

手动了但方向不对，调插件面板上的三个滑块：

| 参数 | 先试 |
|---|---|
| offsetY | 90、-90、0 |
| offsetX | 0、180、±90 |
| offsetZ | 调完前两个再微调 |

调到手指弯曲方向和真手一致。

## 关键文件

| 路径 | 作用 |
|---|---|
| `docs\索引.md` | 所有文档的入口，按问题找 |
| `vam-plugin\README.md` | 插件安装与调参 |
| `docs\vam-协议.md` | UDP 包格式，改协议时两边都要动 |
| `tools\eic_bridge.py` | 上位机 |
| `tools\eic_hand_view.py` | 不看 VaM 也能看手 |

## 已知坑

**QMI8658A 没有 800Hz 档位。** 实际用 896.8Hz，别按整数改。

**PlatformIO 平台版本。** `platformio.ini` 里锁的是 `espressif32@6.12.0`。默认的 7.1.3 会拉 IDF 6.1，在 PIO 下缺构建产物，编译不过。改回新版本会复现。

**PIO 的 Python 依赖走镜像。** 第一次编译如果卡在 pip，设 `PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple`。

**磁力计轴向。** 实测朝向是 x+ 右、y+ 前、z+ 上，与 IMU 的 x+ 后、y+ 右、z+ 上不同，驱动里已经转过一次。换板子要重新确认。

**协议有两份实现。** 驱动里内嵌一份（VaM 只能单文件），`EIGloveProtocol.cs` 是给 Linux 测试的副本。改解析规则两处都要改，`tests\check_proto_sync.py` 会检查。

## 验证命令

Windows 上能跑的：

```powershell
python tests\test_hand_view.py      # 上位机到骨骼的全链路
python tests\check_proto_sync.py    # 协议两份实现是否一致
python tools\eic_frame.py           # 帧协议自检
```

固件相关的（`pio run`、`tools\verify.sh`）在 Windows 上也能跑，但那是 Linux 上验过的，非必要不重复。

## 还没做的

| 项 | 卡在哪 |
|---|---|
| 上板读 IMU | 板子没到 |
| 磁椭球标定 | 板子没到 |
| 手感调参 | 板子没到 |
| VaM 插件实编译 | 只能在 Windows |
| SteamVR 驱动 | 没开始，见 `docs\未来目标.md` |

## 下一步优先级

1. VaM 插件跑通，看到虚拟手跟着动（最高优先，验证整条链路）
2. 板子到了烧固件，读 IMU 确认数据
3. 真手套接上位机，替代 `--sim`
4. 调朝向、调平滑，确定手感参数

## 本机验证记录（2026-11-14，Windows）

在 `E:\EIControler-main` 上把 Windows 侧能验的全部跑通，结果全绿：

| 命令 | 结果 |
|---|---|
| `python tools\eic_bridge.py --sim --verbose` | 正常发包，约 182 fps，手指角度在动 |
| `python tests\test_hand_view.py` | 全链路（UDP→解算→正向运动学）全过 |
| `python tests\check_proto_sync.py` | 两份协议实现一致 |
| `python tools\eic_frame.py` | 帧自检通过 |
| `dotnet run --project tests\csharp\EIProtoTest.csproj` | C# 协议解析 10 项全过 |
| `python tools\eic_hand_view.py --save hand.png --from-sim` | 手骨架图正常生成 |

### 本机修的两个问题

1. **工具脚本的根路径写法只在 Linux 生效。** `eic_bridge.py` 和 `eic_hand_view.py` 里用 `__file__.rsplit("/", 2)` 算项目根目录，Windows 下 `__file__` 是反斜杠，直接跑 `python tools\eic_bridge.py` 会报 `No module named 'tools'`。两处都已改成 `os.path.dirname(os.path.dirname(os.path.abspath(__file__)))`。
2. **C# 测试目标框架 net10.0 编译不过。** 本机只有 .NET 6 SDK（6.0.410），`tests\csharp\EIProtoTest.csproj` 已从 net10.0 降到 net6.0。协议解析只用基础类型，改后 10 项全过。

### 仍然没做（和原状态一致）

- **VaM 插件实编译**：本机没装 VaM，`EIGloveDriver.cs` 依赖 VaM 的 API，只能进 VaM 里编译。装好 VaM 后按上面第 2 步做即可。
- **上板读 IMU / 磁标定 / 手感调参**：板子还没到。
