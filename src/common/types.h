#pragma once

/* 跨模块共享的数据结构。只放类型，不放逻辑。 */

#include <stdbool.h>
#include <stdint.h>

#define EI_IMU_COUNT      12
#define EI_FINGER_COUNT   10
#define EI_MAX_BATCH_SAMP 8

/* 单颗 IMU 在一批里的数据。acc/gyro 是芯片原始 int16 值 */
typedef struct {
    uint8_t  count;
    bool     valid;
    int16_t  acc[EI_MAX_BATCH_SAMP][3];
    int16_t  gyro[EI_MAX_BATCH_SAMP][3];
} imu_slot_t;

/* 一批完整的采样结果，十二颗同一时刻 */
typedef struct {
    uint32_t   ts_us;
    imu_slot_t imu[EI_IMU_COUNT];
} raw_batch_t;

/* 送给 io 层打包的姿态帧 */
typedef struct {
    uint32_t ts_ms;
    float    q_hand[4];              /* 手掌绝对姿态 */
    float    q_rel[EI_FINGER_COUNT][4]; /* 手指相对手掌 */
    float    temp[EI_IMU_COUNT];     /* 片内温度，摄氏度 */
} fusion_frame_t;
