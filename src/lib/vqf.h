#pragma once

/* 单颗 IMU 的姿态估计。纯 C，不依赖 ESP-IDF，可在 PC 上编译测试。
 *
 * 走的是 VQF 的三个核心机制：加速度在近惯性系滤波、陀螺零偏在线估计、
 * 磁干扰检测与降权。相对完整版砍掉了离线模式、任意多采样率组合与对外参数接口。
 */

#include <stdbool.h>

#include "quat.h"

#define VQF_GRAVITY 9.80665f

typedef struct {
    /* 姿态与零偏 */
    quat_t q;
    vec3_t gyro_bias;

    /* 近惯性系里低通后的加速度 */
    vec3_t acc_lp;
    bool   acc_lp_ready;

    /* 静止检测 */
    float  rest_time;

    /* 磁力计 */
    bool   mag_enabled;
    bool   mag_ref_ready;
    vec3_t mag_ref_world;   /* 世界系里的磁场方向，单位向量 */
    float  mag_ref_norm;    /* 参考场强 uT */
    float  mag_weight;      /* 0..1，干扰时衰减 */

    /* 参数 */
    float  acc_lp_alpha;
    float  acc_gain;
    float  mag_gain;
    float  bias_gain_moving;
    float  bias_gain_resting;
} vqf_t;

void vqf_init(vqf_t *f, float sample_rate_hz);
void vqf_set_mag_enabled(vqf_t *f, bool enabled);

/* 陀螺与加速度更新，dt 单位为秒 */
void vqf_update(vqf_t *f, const vec3_t *gyro, const vec3_t *acc, float dt);

/* 磁力计更新，与陀螺不同频，dt 是距上次磁更新的时间 */
void vqf_update_mag(vqf_t *f, const vec3_t *mag, float dt);

/* 把当前航向记成「正前方」，之后只输出相对它的姿态 */
void vqf_reset_heading(vqf_t *f);

/* 直接设定零偏，来自 NVS 校准数据 */
void vqf_set_gyro_bias(vqf_t *f, const vec3_t *bias);

bool vqf_is_resting(const vqf_t *f);

/* 以当前磁场方向建立航向参考，进入 RUNNING 时调一次 */
void vqf_capture_mag_reference(vqf_t *f, const vec3_t *mag);
