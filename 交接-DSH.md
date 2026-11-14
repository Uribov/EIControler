# 交接（DSH）

项目已从 Hermes 目录迁移到 DSH，后续由 DSH 会话负责。

## 路径

| 用途 | 路径 |
|---|---|
| 项目根（新） | `E:\dsh\workspace\EIControler` |
| 项目根（旧，只读备份，确认后删） | `D:\Hermes Agent CN Desktop\Project\EIControler` |
| VaM 插件加载副本 | `E:\哈基米V1臻品整合包\vam1.22.0.13\Custom\Scripts\EIGloveDriver.cs` |
| VaM 日志 | `E:\哈基米V1臻品整合包\vam1.22.0.13\BepInEx\LogOutput.log`（或 VaM 信息日志面板） |

## 快速命令

```powershell
cd E:\dsh\workspace\EIControler

# 仿真数据（无硬件时用这个，200Hz，0.25Hz 慢速握拳）
python -u tools/eic_bridge.py --sim --stats

# 真实串口
python -u tools/eic_bridge.py --port COMx --stats

# 手动摆姿势（拖指尖），200Hz 稳定数据
tools\EIControlerPose.exe

# 看数据源的 3D 手（会占用 9001，跑之前先关 VaM 插件）
python tools/eic_hand_view.py --live
```

## 当前位置（2026-11-14 晚）

- VaM 插件 `EIGloveDriver.cs`：**已回滚成只写 `transform.localRotation`**，不再修改 morph/关节/控制器/动画等持久化状态，避免污染人物。
- 插件仍有诊断：`覆盖检测`、`onPreRender N 次，写后差 X 度`、`i2 name=...` 探针。
- `tools/eic_bridge.py`：`--sim` 改成 0.25Hz 慢速握拳；新增 `--smooth`（四元数低通）、`--clamp`（单帧限幅）、`--stats`（每帧跳变统计）。
- `tools/EIControlerPose.exe`：C# WinForms 手部姿态控制器，源码 `tools/EIControlerPose.cs`，用系统 `csc` 编译。
- 旧的 `eic_bridge.py --sim` 进程（19:37 启动，用旧代码）已停止。

## 排查记录（重要）

“手指疯狂抽”的根因：**19:37 启动的旧 `--sim` 进程一直用旧代码发 1.8Hz/143° 的大幅摆动**，改文件对已运行进程无效。重启数据源后 `--stats` 显示 0.1°/帧。

另外验证过：VaM 1.22 的骨骼是 `DAZBone` + `DAZSkinV2` GPU 蒙皮；写 `transform.localRotation` 在 `LateUpdate` + `Camera.onPreRender` 两处，`onPreRender` 写后立即读回差 0.0°，但下一帧会被 VaM 覆盖几十度。曾尝试关 `disableMorph`/`jointRotationDisabled`/`control`，会让蒙皮变麻花且污染人物，已全部回滚。这个覆盖问题还没根治，后续可以继续查（候选：`Camera.onPostRender` 时机、`DAZBone.currentAngles` 接口）。

## 同步插件到 VaM

```powershell
Copy-Item E:\dsh\workspace\EIControler\vam-plugin\EIGloveDriver.cs `
          'E:\哈基米V1臻品整合包\vam1.22.0.13\Custom\Scripts\EIGloveDriver.cs' -Force
```

然后在 VaM 里 Remove + Add Plugin 重新编译。

## 编译校验（无 VaM 时）

`C:\Users\Uribof\AppData\Local\Temp\eiglove_check\` 下有一套 VaM/Unity API 桩，可用 `dotnet build` 先查语法：

```powershell
Copy-Item E:\dsh\workspace\EIControler\vam-plugin\EIGloveDriver.cs `
          C:\Users\Uribof\AppData\Local\Temp\eiglove_check\proj\EIGloveDriver.cs -Force
dotnet build C:\Users\Uribof\AppData\Local\Temp\eiglove_check\proj -v q
```

## 待办

1. 重载插件后确认手能平滑跟随（数据源用新 `--sim` 或 pose 工具）。
2. 若 `覆盖检测` 仍显示几十度差，继续查渲染层覆盖（`onPostRender` 时机 / `DAZBone.currentAngles`）。
3. 确认 `EIGloveDriver` 里 `DIRECT=false`、`smooth=0.7` 的手感，必要时调。
4. 稳定后 `git commit & push`。
