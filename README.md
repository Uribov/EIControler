# EI Controller

体感手套的主控固件。十二颗 IMU 分时挂在一根 SPI 上，片上完成姿态解算，通过 USB 把四元数发给上位机。

ESP32-S3 + 74HC154 译码片选 + 12 颗 QMI8658A + 1 颗 QMC5883P 磁力计。

## 目录

| 目录 | 内容 |
|---|---|
| `src/board` | 引脚定义、SPI 与 I2C 总线、74HC154 片选 |
| `src/drivers` | QMI8658A 与 QMC5883P 的寄存器驱动 |
| `src/lib` | 四元数与 VQF 姿态滤波，纯 C，可在 PC 上编译 |
| `src/sample` | 5ms 周期的扫描调度与进队 |
| `src/fusion` | 十二颗姿态解算、手掌双 IMU 融合、相对姿态 |
| `src/io` | 帧打包与 USB 发送 |
| `src/config` | NVS 里的零偏与校准数据 |
| `tests` | PC 端算法测试 |
| `tools` | 上位机侧协议工具 |
| `docs` | 架构与各模块的实现契约 |

## 构建

```bash
pio run
```

产物在 `.pio/build/eicontroller/`，烧录：

```bash
pio run -t upload
```

首次烧录建议整片擦除，避免旧的 NVS 配置残留：

```bash
pio run -t erase && pio run -t upload
```

## 硬件连接

SPI 与片选：

| 信号 | GPIO |
|---|---|
| SCK | 13 |
| MOSI | 12 |
| MISO | 11 |
| A0 / A1 / A2 / A3 | 8 / 9 / 10 / 5 |
| CS_E | 4 |

磁力计走 I2C，SDA=7、SCL=6，地址 `0x2C`。

十二颗 IMU 接 74HC154 的 Y0 到 Y11，对应片选索引 0 到 11。Y12 到 Y15 空置。

## 上位机协议

USB 虚拟串口输出定长帧，每帧 210 字节，小端：

| 字段 | 长度 | 说明 |
|---|---|---|
| 帧头 | 2 | `0xAA 0x55` |
| 序号 | 2 | 每帧加一，回绕 |
| 时间戳 | 4 | 毫秒 |
| 手掌四元数 | 16 | 4 个 float，顺序 `w x y z` |
| 相对四元数 | 160 | 10 × 4 float，手指依次排列 |
| 温度 | 24 | 12 × int16，单位 0.01 摄氏度 |
| CRC | 2 | CRC16-CCITT-FALSE，覆盖前 208 字节 |

解析工具：

```bash
python3 tools/eic_frame.py            # 协议自检
python3 tools/eic_frame.py --dump x.bin
```

上位机与可视化：

```bash
python3 tools/eic_bridge.py --sim              # 合成数据发 UDP
python3 tools/eic_bridge.py --port /dev/ttyACM0
python3 tools/eic_hand_view.py --live          # 实时看手
python3 tools/eic_hand_view.py --save hand.png --from-sim
```

手指与片选的对应关系见 `docs/fusion.md`，左右手不同。

## 当前状态

| 项 | 状态 |
|---|---|
| 全部模块编译 | 通过，Flash 占用 9.6% |
| 姿态算法 | PC 端五个场景测试通过 |
| 帧协议 | C 与 Python 双向对拍通过 |
| 上板读 IMU | 待硬件 |
| 磁标定与手感调参 | 待硬件 |

## 文档

- `交接-Windows.md`：**Windows 侧先看这个**，列了上手顺序和已知坑
- `docs/索引.md`：按问题找文档的入口，含全部关键参数
- `docs/架构.md`：分层、任务、时序预算、决策记录
- `docs/` 下其余各模块文档：接口契约与实现说明
- `docs/交付说明.md`：做完的部分、验证方法、待硬件清单

文档里的接口签名是契约，改代码前先改文档。
