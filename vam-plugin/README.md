# VaM 插件

把手套数据套到 VaM 人体骨骼上。

## 文件

| 文件 | 用途 |
|---|---|
| `EIGloveDriver.cs` | 插件主体，放进 VaM 的 `Custom/Scripts/` |
| `EIGloveProtocol.cs` | 协议解析的独立副本，只用于 Linux 上跑测试，不用放进 VaM |

VaM 的脚本只能单文件，所以驱动里内嵌了协议解析。两份要保持同步，`tests/check_proto_sync.py` 会检查。

## 安装

1. 把 `EIGloveDriver.cs` 放到 VaM 目录下的 `Custom/Scripts/`
2. 场景里选中要控制的人体 Atom
3. Add Plugin → 选 `EIGloveDriver`
4. 启动上位机：

```bash
python3 tools/eic_bridge.py --port /dev/ttyACM0 --hand 0
```

## 面板参数

| 参数 | 范围 | 作用 |
|---|---|---|
| enabled | 开关 | 总开关 |
| rightHand | 开关 | 手套戴哪只手，勾上控制角色右手 |
| strength | 0 到 1.5 | 混合强度，0 回到原始姿势，1 全量 |
| offsetX / offsetY / offsetZ | -180 到 180 | 骨骼朝向补偿，见下文 |
| smooth | 0 到 0.95 | 关节平滑，压 IMU 噪声，0 不平滑 |

## 朝向对不上怎么调

手套机体系和 VaM 骨骼朝向不一致，用三个 offset 角度补。

调的顺序：

1. 先调 **offsetY**，这是绕竖直轴的旋转，一般是 90 或 -90
2. 再看手指是朝手背弯还是朝掌心弯，不对就调 **offsetX**
3. 最后微调 **offsetZ**

调到手指弯曲方向和真实手一致就对了。

默认值 `(0, 90, 0)` 是猜的起点，基本要改。

## 断流保护

超过 0.6 秒没收到包，强度会平滑降到 0，虚拟手回到原始姿势，不会僵在最后一帧。

## 诊断日志

插件每 5 秒往 VaM 日志打一行：

```
[EIGlove] 收包 198/s，累计 1200，坏包 0
```

收不到包时会提示检查端口。

如果 16 个骨骼一个都没匹配上，日志会把人物身上所有手部相关的骨骼名列出来，照着改 `BoneNamesRight` 数组即可（DAZ Genesis 8 的命名是 `rHand`、`rThumb1` 这一套，不同人物可能不一样）。

## 没有 VaM 怎么验证

Linux 上能验的部分：

```bash
bash tools/verify.sh          # 含 C# 协议解析测试与两份实现的同步检查
python3 tools/eic_hand_view.py --live    # 看虚拟手
```

真正套到骨骼上还是得在 VaM 里看。

## 当前状态

| 项 | 状态 |
|---|---|
| UDP 收发 | 上位机侧已验证 |
| C# 协议解析 | 用 dotnet 单测过，10 项全过 |
| 骨骼映射 | 按 DAZ G8 命名写的，需现场核对 |
| 朝向补偿 | 三个滑块已做，值要实调 |
| 整体编译 | 未在 VaM 里跑过 |
