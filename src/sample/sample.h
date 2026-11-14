#pragma once

#include "common/types.h"
#include "esp_err.h"

typedef struct {
    uint32_t drop_batch;   /* 队列满丢整批 */
    uint32_t drop_imu;     /* 单颗读失败 */
    uint32_t overrun;      /* FIFO 溢出或样本数超上限 */
    uint32_t comm_err;     /* SPI 无应答 */
    uint32_t samples_total;
} sample_stats_t;

esp_err_t sample_start(void);

/* 阻塞取一批，超时返回 false */
bool sample_fetch(raw_batch_t *out, uint32_t timeout_ms);

/* 最新一轮的十二颗片内温度，摄氏度 */
void sample_get_temps(float out[EI_IMU_COUNT]);

sample_stats_t sample_get_stats(void);
