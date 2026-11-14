#include "fusion.h"

#include <math.h>
#include <string.h>

#include "config/config.h"
#include "esp_log.h"
#include "drivers/qmc5883p.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lib/vqf.h"

static const char *TAG = "fusion";

#define NOMINAL_RATE   896.8f
#define NOMINAL_DT     (1.0f / NOMINAL_RATE)
#define PI_F           3.14159265358979f

/* 陀螺 ±1024dps、加速度 ±8g，换算到 rad/s 与 m/s^2 */
#define GYRO_LSB  ((1024.0f / 32768.0f) * (PI_F / 180.0f))
#define ACC_LSB   ((8.0f / 32768.0f) * 9.80665f)

#define HAND_IDX   EI_FINGER_COUNT  /* vqf 数组里手掌占最后一个 */
#define VQF_COUNT  (EI_FINGER_COUNT + 1)

#define BIAS_SAMPLES_TARGET 500u

static vqf_t s_vqf[VQF_COUNT];
static fusion_state_t s_state = FUSION_IDLE;
static bool s_inited;

static uint32_t s_prev_ts_us;
static float s_last_dt;

/* 校准阶段累加的原始读数 */
static vec3_t s_bias_sum[EI_FINGER_COUNT + 1];
static uint32_t s_bias_samples;
static bool s_bias_done[EI_FINGER_COUNT + 1];

static void mag_task(void *arg);

static vec3_t decode_gyro(const int16_t v[3])
{
    const vec3_t g = { v[0] * GYRO_LSB, v[1] * GYRO_LSB, v[2] * GYRO_LSB };
    return g;
}

static vec3_t decode_acc(const int16_t v[3])
{
    const vec3_t a = { v[0] * ACC_LSB, v[1] * ACC_LSB, v[2] * ACC_LSB };
    return a;
}

esp_err_t fusion_init(void)
{
    for (int i = 0; i < VQF_COUNT; i++) {
        vqf_init(&s_vqf[i], NOMINAL_RATE);

        float bias[3] = { 0.0f, 0.0f, 0.0f };
        if (cfg_get_gyro_bias((uint8_t)((i == HAND_IDX) ? 10 : i), bias) == ESP_OK) {
            const vec3_t b = { bias[0], bias[1], bias[2] };
            vqf_set_gyro_bias(&s_vqf[i], &b);
        }
    }

    /* 手掌那颗走 9D，手指走 6D。磁力计初始化失败时自动退回 6D */
    if (qmc_init() == ESP_OK) {
        vqf_set_mag_enabled(&s_vqf[HAND_IDX], true);
    } else {
        ESP_LOGW(TAG, "磁力计不可用，手掌退化为 6D");
        vqf_set_mag_enabled(&s_vqf[HAND_IDX], false);
    }

    memset(s_bias_sum, 0, sizeof(s_bias_sum));
    memset(s_bias_done, 0, sizeof(s_bias_done));
    s_bias_samples = 0;
    s_prev_ts_us = 0;
    s_last_dt = NOMINAL_DT;
    s_state = FUSION_IDLE;
    s_inited = true;

    xTaskCreate(mag_task, "mag", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "init done, %d 实例", VQF_COUNT);
    return ESP_OK;
}

/* 磁力计 100Hz，只喂手掌那颗。读失败就跳过，不影响六轴 */
static void mag_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        vec3_t m;
        if (qmc_read(&m) == ESP_OK) {
            vqf_update_mag(&s_vqf[HAND_IDX], &m, 0.01f);
        }
    }
}

/* 把一颗的数据按样本逐个喂进对应的滤波器 */
static void feed_slot(vqf_t *f, const imu_slot_t *slot, float dt)
{
    if (!slot->valid || slot->count == 0) {
        return;
    }

    for (uint8_t k = 0; k < slot->count; k++) {
        const vec3_t g = decode_gyro(slot->gyro[k]);
        const vec3_t a = decode_acc(slot->acc[k]);
        vqf_update(f, &g, &a, dt);
    }
}

/* 手掌两颗加权融合后喂同一个实例。逆方差加权在噪声相近时退化为等权平均 */
static void feed_hand(vqf_t *f, const imu_slot_t *a, const imu_slot_t *b, float dt)
{
    const bool va = a->valid && a->count > 0;
    const bool vb = b->valid && b->count > 0;

    if (!va && !vb) {
        return;
    }
    if (va && !vb) {
        feed_slot(f, a, dt);
        return;
    }
    if (!va && vb) {
        feed_slot(f, b, dt);
        return;
    }

    const uint8_t n = (a->count < b->count) ? a->count : b->count;
    for (uint8_t k = 0; k < n; k++) {
        const vec3_t ga = decode_gyro(a->gyro[k]);
        const vec3_t gb = decode_gyro(b->gyro[k]);
        const vec3_t aa = decode_acc(a->acc[k]);
        const vec3_t ab = decode_acc(b->acc[k]);

        const vec3_t g = { (ga.x + gb.x) * 0.5f, (ga.y + gb.y) * 0.5f, (ga.z + gb.z) * 0.5f };
        const vec3_t ac = { (aa.x + ab.x) * 0.5f, (aa.y + ab.y) * 0.5f, (aa.z + ab.z) * 0.5f };
        vqf_update(f, &g, &ac, dt);
    }
}

static void collect_bias(const raw_batch_t *b)
{
    if (s_bias_samples >= BIAS_SAMPLES_TARGET) {
        return;
    }

    for (int i = 0; i < VQF_COUNT; i++) {
        const uint8_t src = (i == HAND_IDX) ? 10 : (uint8_t)i;
        const imu_slot_t *slot = &b->imu[src];
        if (!slot->valid || slot->count == 0) {
            continue;
        }
        for (uint8_t k = 0; k < slot->count; k++) {
            const vec3_t g = decode_gyro(slot->gyro[k]);
            s_bias_sum[i] = vec3_add(&s_bias_sum[i], &g);
        }
    }
    s_bias_samples++;
}

void fusion_process_batch(const raw_batch_t *batch)
{
    if (!s_inited || batch == NULL) {
        return;
    }

    /* 用批间时间戳差算 dt，兼容 ODR 的实际偏差 */
    float dt = s_last_dt;
    if (s_prev_ts_us != 0 && batch->ts_us > s_prev_ts_us) {
        const uint32_t span = batch->ts_us - s_prev_ts_us;
        uint8_t n = 1;
        for (int i = 0; i < VQF_COUNT; i++) {
            if (batch->imu[i].count > n) {
                n = batch->imu[i].count;
            }
        }
        if (span > 0) {
            dt = ((float)span * 1e-6f) / (float)n;
        }
    }
    s_prev_ts_us = batch->ts_us;
    s_last_dt = dt;

    if (s_state == FUSION_GYRO_BIAS) {
        collect_bias(batch);
        if (s_bias_samples >= BIAS_SAMPLES_TARGET) {
            for (int i = 0; i < VQF_COUNT; i++) {
                const float inv = 1.0f / (float)s_bias_samples;
                const vec3_t b = {
                    s_bias_sum[i].x * inv,
                    s_bias_sum[i].y * inv,
                    s_bias_sum[i].z * inv
                };
                vqf_set_gyro_bias(&s_vqf[i], &b);
                s_bias_done[i] = true;
            }
            ESP_LOGI(TAG, "零偏采样完成，%u 个样本", (unsigned)s_bias_samples);
            s_state = FUSION_YAW_ZERO;
        }
    }

    for (int i = 0; i < EI_FINGER_COUNT; i++) {
        feed_slot(&s_vqf[i], &batch->imu[i], dt);
    }

    feed_hand(&s_vqf[HAND_IDX], &batch->imu[10], &batch->imu[11], dt);

    if (s_state == FUSION_YAW_ZERO) {
        vqf_reset_heading(&s_vqf[HAND_IDX]);
        s_state = FUSION_RUNNING;
        ESP_LOGI(TAG, "进入 RUNNING");
    }
}

void fusion_get_frame(fusion_frame_t *out, const float temps[EI_IMU_COUNT])
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));

    const quat_t qh = s_vqf[HAND_IDX].q;
    out->q_hand[0] = qh.w;
    out->q_hand[1] = qh.x;
    out->q_hand[2] = qh.y;
    out->q_hand[3] = qh.z;

    const quat_t qh_inv = quat_conj(&qh);
    for (int i = 0; i < EI_FINGER_COUNT; i++) {
        const quat_t rel = quat_mul(&qh_inv, &s_vqf[i].q);
        out->q_rel[i][0] = rel.w;
        out->q_rel[i][1] = rel.x;
        out->q_rel[i][2] = rel.y;
        out->q_rel[i][3] = rel.z;
    }

    if (temps != NULL) {
        memcpy(out->temp, temps, sizeof(out->temp));
    }

    out->ts_ms = s_prev_ts_us / 1000u;
}

void fusion_calib_begin(void)
{
    memset(s_bias_sum, 0, sizeof(s_bias_sum));
    memset(s_bias_done, 0, sizeof(s_bias_done));
    s_bias_samples = 0;
    s_state = FUSION_GYRO_BIAS;
    ESP_LOGI(TAG, "开始零偏采样");
}

fusion_state_t fusion_state(void)
{
    return s_state;
}

esp_err_t fusion_finish_calibration(void)
{
    for (int i = 0; i < VQF_COUNT; i++) {
        const uint8_t idx = (uint8_t)((i == HAND_IDX) ? 10 : i);
        const float b[3] = {
            s_vqf[i].gyro_bias.x,
            s_vqf[i].gyro_bias.y,
            s_vqf[i].gyro_bias.z
        };
        (void)cfg_set_gyro_bias(idx, b);
    }
    return cfg_save_all();
}
