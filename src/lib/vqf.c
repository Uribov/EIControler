#include "vqf.h"

#include <math.h>
#include <string.h>

/* 静止判据 */
#define REST_GYRO_MAX      0.05f    /* rad/s */
#define REST_ACC_TOL       0.30f    /* m/s^2 */
#define REST_TIME_NEEDED   0.5f     /* s */

/* 倾角校正的最大单步旋转，防止快速冲击把姿态拽飞 */
#define TILT_MAX_STEP      0.15f    /* rad */
#define TILT_MAX_RATE      1.0f     /* rad/s */

/* 磁干扰判据 */
#define MAG_NORM_TOL       0.30f    /* 模长偏离 30% 判干扰 */
#define MAG_DIST_RECOVER   0.20f    /* 每秒恢复的权重 */

void vqf_init(vqf_t *f, float sample_rate_hz)
{
    memset(f, 0, sizeof(*f));
    f->q = quat_identity();
    f->mag_enabled = false;
    f->mag_weight = 1.0f;

    /* 加速度低通：截止约 1Hz，与采样率无关地换算系数 */
    const float tau = 0.15f;
    f->acc_lp_alpha = 1.0f - expf(-1.0f / (tau * sample_rate_hz));
    if (f->acc_lp_alpha > 1.0f) {
        f->acc_lp_alpha = 1.0f;
    }

    f->acc_gain = 0.02f;
    f->mag_gain = 0.01f;
    f->bias_gain_moving = 5e-6f;   /* 时间常数约 220 秒，只跟温漂 */
    f->bias_gain_resting = 0.01f;
}

void vqf_set_mag_enabled(vqf_t *f, bool enabled)
{
    f->mag_enabled = enabled;
    if (!enabled) {
        f->mag_ref_ready = false;
    }
}

void vqf_set_gyro_bias(vqf_t *f, const vec3_t *bias)
{
    f->gyro_bias = *bias;
}

bool vqf_is_resting(const vqf_t *f)
{
    return f->rest_time >= REST_TIME_NEEDED;
}

void vqf_reset_heading(vqf_t *f)
{
    /* 构造一个只含航向的旋转，把它乘到当前姿态上，使当前朝向变成零度 */
    const float yaw = atan2f(2.0f * (f->q.w * f->q.z + f->q.x * f->q.y),
                             1.0f - 2.0f * (f->q.y * f->q.y + f->q.z * f->q.z));
    const float half = -0.5f * yaw;
    const quat_t corr = { cosf(half), 0.0f, 0.0f, sinf(half) };
    f->q = quat_normalize(quat_mul(&corr, &f->q));

    if (f->mag_ref_ready) {
        /* 磁参考同样转到新的世界系 */
        f->mag_ref_world = quat_rotate(&corr, &f->mag_ref_world);
    }
}

void vqf_update(vqf_t *f, const vec3_t *gyro, const vec3_t *acc, float dt)
{
    if (dt <= 0.0f) {
        return;
    }

    /* 1. 扣零偏后积分 */
    const vec3_t w = vec3_sub(gyro, &f->gyro_bias);
    const quat_t dq = quat_from_gyro(&w, dt);
    quat_t q_gyr = quat_normalize(quat_mul(&f->q, &dq));

    /* 2. 加速度转到世界系并低通 */
    const vec3_t a_w = quat_rotate(&q_gyr, acc);
    if (!f->acc_lp_ready) {
        f->acc_lp = a_w;
        f->acc_lp_ready = true;
    } else {
        const vec3_t d = vec3_sub(&a_w, &f->acc_lp);
        const vec3_t step = vec3_scale(&d, f->acc_lp_alpha);
        f->acc_lp = vec3_add(&f->acc_lp, &step);
    }

    /* 3. 用滤波后的世界系加速度求倾角修正 */
    const float a_norm = vec3_norm(&f->acc_lp);
    if (a_norm > 1e-3f) {
        const vec3_t a_hat = vec3_scale(&f->acc_lp, 1.0f / a_norm);
        const vec3_t up = { 0.0f, 0.0f, 1.0f };

        /* 只在偏离重力方向时修正，且限制单步幅度 */
        quat_t q_err = quat_from_two_vectors(&a_hat, &up);
        float angle = 2.0f * acosf(fminf(1.0f, fabsf(q_err.w)));
        if (angle > TILT_MAX_STEP) {
            const float s = TILT_MAX_STEP / angle;
            q_err = quat_slerp(&(quat_t){ 1.0f, 0.0f, 0.0f, 0.0f }, &q_err, s);
        }
        const float rate = TILT_MAX_RATE * dt;
        float gain = f->acc_gain;
        if (gain > rate) {
            gain = rate;
        }
        const quat_t identity = quat_identity();
        const quat_t partial = quat_slerp(&identity, &q_err, gain);
        q_gyr = quat_normalize(quat_mul(&partial, &q_gyr));
    }

    f->q = q_gyr;

    /* 4. 静止检测与零偏估计 */
    const float w_norm = vec3_norm(&w);
    const float a_dev = fabsf(vec3_norm(acc) - VQF_GRAVITY);
    if (w_norm < REST_GYRO_MAX && a_dev < REST_ACC_TOL) {
        f->rest_time += dt;
    } else {
        f->rest_time = 0.0f;
    }

    const float gain = (f->rest_time >= REST_TIME_NEEDED) ? f->bias_gain_resting
                                                          : f->bias_gain_moving;
    const vec3_t d = vec3_sub(gyro, &f->gyro_bias);
    const vec3_t bstep = vec3_scale(&d, gain);
    f->gyro_bias = vec3_add(&f->gyro_bias, &bstep);

    f->q = quat_normalize(f->q);
}

void vqf_update_mag(vqf_t *f, const vec3_t *mag, float dt)
{
    if (!f->mag_enabled) {
        return;
    }

    const float n = vec3_norm(mag);
    if (n < 1e-3f) {
        return;
    }

    /* 还没有参考时先建立参考，本次不校正 */
    if (!f->mag_ref_ready) {
        vqf_capture_mag_reference(f, mag);
        return;
    }

    /* 磁干扰：模长偏离参考过多时降权，恢复靠时间 */
    const float dev = fabsf(n - f->mag_ref_norm) / f->mag_ref_norm;
    if (dev > MAG_NORM_TOL) {
        f->mag_weight = 0.0f;
    } else {
        f->mag_weight = fminf(1.0f, f->mag_weight + MAG_DIST_RECOVER * dt);
    }

    if (f->mag_weight <= 0.0f) {
        return;
    }

    /* 把磁场转到世界系，取其水平分量作为航向参考 */
    const vec3_t m_w = quat_rotate(&f->q, mag);
    const float m_h = sqrtf(m_w.x * m_w.x + m_w.y * m_w.y);
    if (m_h < 1e-3f) {
        return;
    }

    const float yaw_meas = atan2f(m_w.y, m_w.x);

    const float ref_yaw = atan2f(f->mag_ref_world.y, f->mag_ref_world.x);

    float err = ref_yaw - yaw_meas;
    while (err > 3.14159265f) {
        err -= 6.2831853f;
    }
    while (err < -3.14159265f) {
        err += 6.2831853f;
    }

    const float step = f->mag_gain * f->mag_weight * err;
    const float half = 0.5f * step;
    const quat_t corr = { cosf(half), 0.0f, 0.0f, sinf(half) };
    f->q = quat_normalize(quat_mul(&corr, &f->q));
}

void vqf_capture_mag_reference(vqf_t *f, const vec3_t *mag)
{
    if (!f->mag_enabled) {
        return;
    }
    const float n = vec3_norm(mag);
    if (n < 1e-3f) {
        return;
    }
    const vec3_t m_w = quat_rotate(&f->q, mag);
    const float m_h = sqrtf(m_w.x * m_w.x + m_w.y * m_w.y);
    if (m_h < 1e-3f) {
        return;
    }
    f->mag_ref_world.x = m_w.x / m_h;
    f->mag_ref_world.y = m_w.y / m_h;
    f->mag_ref_world.z = 0.0f;
    f->mag_ref_norm = n;
    f->mag_ref_ready = true;
    f->mag_weight = 1.0f;
}
