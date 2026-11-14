#pragma once

#include "common/types.h"
#include "esp_err.h"

/* 校准状态 */
typedef enum {
    FUSION_IDLE = 0,
    FUSION_GYRO_BIAS,
    FUSION_YAW_ZERO,
    FUSION_RUNNING,
} fusion_state_t;

esp_err_t fusion_init(void);

/* 处理一批原始样本。内部按批更新 11 个 VQF 实例 */
void fusion_process_batch(const raw_batch_t *batch);

/* 取最新一帧姿态。温度由 sample 层填好再传进来 */
void fusion_get_frame(fusion_frame_t *out, const float temps[EI_IMU_COUNT]);

/* 校准控制 */
void fusion_calib_begin(void);
fusion_state_t fusion_state(void);

/* 把当前航向记成零度，并落盘零偏 */
esp_err_t fusion_finish_calibration(void);
