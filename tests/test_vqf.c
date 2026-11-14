/* PC 端姿态算法离线测试。用 gcc 编译，不依赖 ESP-IDF。
 *
 * 编译： gcc -O2 -o /tmp/test_vqf tests/test_vqf.c src/lib/vqf.c -lm
 * 运行： /tmp/test_vqf
 */

#include <math.h>
#include <stdio.h>

#include "../src/lib/vqf.h"

#define DT (1.0f / 896.8f)

static void print_q(const char *tag, const quat_t *q)
{
    printf("%s q=(%+.4f %+.4f %+.4f %+.4f) |q|=%.6f\n", tag, q->w, q->x, q->y, q->z,
           quat_norm(q));
}

/* 从四元数取欧拉角，单位度 */
static void quat_to_euler(const quat_t *q, float *roll, float *pitch, float *yaw)
{
    const float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    const float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp);

    const float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    const float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny_cosp, cosy_cosp);

    *roll *= 57.29578f;
    *pitch *= 57.29578f;
    *yaw *= 57.29578f;
}

static int fail_count;

static void expect_near(const char *name, float got, float want, float tol)
{
    const float d = fabsf(got - want);
    const int ok = d <= tol;
    printf("  [%s] %-28s got=%+8.3f want=%+8.3f tol=%.3f\n", ok ? "PASS" : "FAIL", name,
           got, want, tol);
    if (!ok) {
        fail_count++;
    }
}

/* 场景 1：静止。姿态应保持单位四元数，零偏收敛到零 */
static void test_static(void)
{
    printf("\n== 场景 1：静止 10 秒 ==\n");
    vqf_t f;
    vqf_init(&f, 896.8f);

    const vec3_t gyro = { 0.0f, 0.0f, 0.0f };
    const vec3_t acc = { 0.0f, 0.0f, 9.80665f };

    for (int i = 0; i < (int)(10.0f / DT); i++) {
        vqf_update(&f, &gyro, &acc, DT);
    }

    print_q("  结果", &f.q);
    expect_near("roll", 0.0f, 0.0f, 0.5f);
    expect_near("w 分量", f.q.w, 1.0f, 1e-3f);
    expect_near("零偏 x", f.gyro_bias.x, 0.0f, 1e-4f);
}

/* 场景 2：绕 z 轴匀速转 90 度，检查积分角度 */
static void test_yaw_rotation(void)
{
    printf("\n== 场景 2：绕 z 轴 1 秒转 90 度 ==\n");
    vqf_t f;
    vqf_init(&f, 896.8f);

    const float rate = (float)(M_PI / 2.0); /* rad/s */
    const vec3_t gyro = { 0.0f, 0.0f, rate };
    const vec3_t acc = { 0.0f, 0.0f, 9.80665f };

    for (int i = 0; i < (int)(1.0f / DT); i++) {
        vqf_update(&f, &gyro, &acc, DT);
    }

    float r, p, y;
    quat_to_euler(&f.q, &r, &p, &y);
    print_q("  结果", &f.q);
    expect_near("yaw", y, 90.0f, 2.0f);
    expect_near("roll", r, 0.0f, 1.0f);
    expect_near("pitch", p, 0.0f, 1.0f);
}

/* 场景 3：姿态初始错 20 度，喂重力后应收敛 */
static void test_tilt_convergence(void)
{
    printf("\n== 场景 3：初始倾角偏 20 度，静止收敛 ==\n");
    vqf_t f;
    vqf_init(&f, 896.8f);

    /* 人为给一个 roll = 20 度的初始姿态 */
    const float half = 0.5f * 20.0f / 57.29578f;
    const quat_t q0 = { cosf(half), sinf(half), 0.0f, 0.0f };
    f.q = q0;

    const vec3_t gyro = { 0.0f, 0.0f, 0.0f };
    /* IMU 实际平放，加速度计读到的就是机体系里的重力分量 */
    const vec3_t acc = { 0.0f, 0.0f, 9.80665f };

    for (int i = 0; i < (int)(3.0f / DT); i++) {
        vqf_update(&f, &gyro, &acc, DT);
    }

    float r, p, y;
    quat_to_euler(&f.q, &r, &p, &y);
    print_q("  结果", &f.q);
    expect_near("roll 收敛到 0", r, 0.0f, 3.0f);
}

/* 场景 4：零偏注入，静止时应被估掉 */
static void test_bias_estimation(void)
{
    printf("\n== 场景 4：注入 0.02 rad/s 零偏，静止 20 秒 ==\n");
    vqf_t f;
    vqf_init(&f, 896.8f);

    const vec3_t gyro = { 0.02f, -0.015f, 0.01f };
    const vec3_t acc = { 0.0f, 0.0f, 9.80665f };

    for (int i = 0; i < (int)(20.0f / DT); i++) {
        vqf_update(&f, &gyro, &acc, DT);
    }

    float r, p, y;
    quat_to_euler(&f.q, &r, &p, &y);
    printf("  估计零偏 = (%+.5f %+.5f %+.5f) rad/s\n", f.gyro_bias.x, f.gyro_bias.y,
           f.gyro_bias.z);
    expect_near("bias x", f.gyro_bias.x, 0.02f, 5e-3f);
    expect_near("bias y", f.gyro_bias.y, -0.015f, 5e-3f);
    expect_near("roll", r, 0.0f, 3.0f);
}

/* 场景 5：航向零点。转过去再按一下，应回到零 */
static void test_heading_reset(void)
{
    printf("\n== 场景 5：转 45 度后设航向零点 ==\n");
    vqf_t f;
    vqf_init(&f, 896.8f);

    const vec3_t gyro = { 0.0f, 0.0f, (float)(M_PI / 4.0) };
    const vec3_t acc = { 0.0f, 0.0f, 9.80665f };

    for (int i = 0; i < (int)(1.0f / DT); i++) {
        vqf_update(&f, &gyro, &acc, DT);
    }

    float r, p, y;
    quat_to_euler(&f.q, &r, &p, &y);
    printf("  转后 yaw = %+.2f 度\n", y);

    vqf_reset_heading(&f);
    quat_to_euler(&f.q, &r, &p, &y);
    expect_near("零点后 yaw", y, 0.0f, 0.5f);
}

int main(void)
{
    test_static();
    test_yaw_rotation();
    test_tilt_convergence();
    test_bias_estimation();
    test_heading_reset();

    printf("\n===== %s（失败 %d 项）=====\n", fail_count == 0 ? "全部通过" : "有失败",
           fail_count);
    return fail_count == 0 ? 0 : 1;
}
