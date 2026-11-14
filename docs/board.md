# board —— 引脚与总线初始化

## 职责

把硬件资源准备好，给上层一层干净的接口：

- 引脚定义集中在一处，换板只改这一个文件
- 初始化 SPI 总线、I2C 总线、74HC154 的地址线、`esp_timer`
- 提供片选切换函数，屏蔽 74HC154 的时序细节

不碰任何寄存器语义，也不做数据处理。上层拿到的是「总线句柄」和「选通第几颗」两个概念。

## 依赖

- ESP-IDF 的 `driver/spi_master`、`driver/i2c`、`driver/gpio`、`esp_timer`
- 无其它模块依赖，`board` 是最底层

## 对外接口

```c
// 初始化全部硬件资源
esp_err_t board_init(void);

// 片选第 idx 颗 IMU，idx 0..11
void board_select(uint8_t idx);

// 取消片选
void board_deselect(void);

// 总线句柄
spi_device_handle_t board_spi(void);
i2c_master_bus_handle_t board_i2c(void);

// 5ms 周期定时器，回调里只发通知
esp_err_t board_timer_start(void (*cb)(void *arg), void *arg);
```

`board_select` 内部处理了 74HC154 的禁止-切换-使能顺序，上层直接调用即可。

## 验收（P0）

- `board_init` 返回成功，`esp_timer` 能起 5ms 周期
- 十二个 idx 逐个选通，用逻辑分析仪能看到对应引脚的 CS 依次拉低
- 选通后 SPI 能读到 QMI8658A 的 `WHO_AM_I`
- I2C 能读到磁力计的 `CHIP_ID`

第二条是硬指标。十二路片选只要有一路错位，那一颗就永远读不到。

## 当前实现架构（尚未实现）

### 简介

`board` 只做两件事：把引脚定义说清楚，把总线拉起来。它不关心 IMU 里有什么寄存器，也不关心数据怎么用。

### 引脚表

| 信号 | GPIO | 方向 | 说明 |
|---|---|---|---|
| SCK | 13 | 输出 | SPI 时钟 |
| MOSI | 12 | 输出 | 主机发 |
| MISO | 11 | 输入 | 主机收 |
| A0 | 8 | 输出 | 74HC154 地址位 0 |
| A1 | 9 | 输出 | 地址位 1 |
| A2 | 10 | 输出 | 地址位 2 |
| A3 | 5 | 输出 | 地址位 3 |
| CS_E | 4 | 输出 | 74HC154 使能，低有效 |
| SDA | 7 | 双向 | I2C 数据，外接上拉 |
| SCL | 6 | 输出 | I2C 时钟，外接上拉 |

引脚以原理图为准。所有编号只在 `pinmap.h` 里出现一次，其它文件一律引用宏。

### SPI 总线

用 SPI2_HOST，15MHz，模式 0，DMA 自动分配。

设备配置里 `spics_io_num` 设 -1，因为片选不走硬件 CS 引脚，由 74HC154 译码产生。每条事务的 CS 由软件显式控制。

这样做的代价是每次传输前要自己切片选，好处是十二条通道共用一套 SPI 配置。

### 片选切换顺序

切换通道时必须先禁用再改地址，否则地址线在变化过程中可能短暂选中别的通道：

```
1. CS_E 拉高        （禁用译码器输出）
2. 写 A0 到 A3      （切换地址）
3. CS_E 拉低        （使能目标通道）
```

`board_select(idx)` 就是把这三步打包，`board_deselect` 只做第一步。

中间态那几十纳秒如果有毛刺，被选中的那颗会收到半个时钟，不过是空事务，不会写坏寄存器。

### I2C 总线

用 I2C_NUM_0，400kHz，主机模式。

磁力计的地址由 SA0 引脚决定，原理图上拉还是接地要看实际连接。驱动里两个地址都试一次，哪个能读出 `CHIP_ID` 就用哪个。

### 定时器

`board_timer_start` 起一个 5ms 周期的 `esp_timer`，回调里只调用传入的函数指针。

回调的调度方式用 `ESP_TIMER_TASK`，也就是跑在 `esp_timer` 自己那个任务里，不是中断上下文。这样回调里可以调 FreeRTOS 的接口，`sample` 传进来的 `xTaskNotifyGive` 也就能用。

即便如此，回调里仍然不能做 SPI 传输这类耗时操作，只发通知，活儿留给采集任务。
