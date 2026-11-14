#pragma once

/* 四元数与向量基础运算。纯 C，不依赖 ESP-IDF，可在 PC 上直接编译测试。 */

#include <math.h>
#include <stdbool.h>

typedef struct {
    float w, x, y, z;
} quat_t;

typedef struct {
    float x, y, z;
} vec3_t;

static inline quat_t quat_identity(void)
{
    const quat_t q = { 1.0f, 0.0f, 0.0f, 0.0f };
    return q;
}

static inline float quat_norm(const quat_t *q)
{
    return sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

static inline quat_t quat_normalize(quat_t q)
{
    const float n = quat_norm(&q);
    if (n < 1e-6f) {
        return quat_identity();
    }
    const float inv = 1.0f / n;
    q.w *= inv;
    q.x *= inv;
    q.y *= inv;
    q.z *= inv;
    return q;
}

/* Hamilton 乘法：a ⊗ b */
static inline quat_t quat_mul(const quat_t *a, const quat_t *b)
{
    const quat_t r = {
        a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z,
        a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y,
        a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x,
        a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w
    };
    return r;
}

static inline quat_t quat_conj(const quat_t *q)
{
    const quat_t r = { q->w, -q->x, -q->y, -q->z };
    return r;
}

/* 机体系向量转到世界系：v_w = R(q) · v_b */
static inline vec3_t quat_rotate(const quat_t *q, const vec3_t *v)
{
    const quat_t p = { 0.0f, v->x, v->y, v->z };
    const quat_t qc = quat_conj(q);
    const quat_t t = quat_mul(q, &p);
    const quat_t r = quat_mul(&t, &qc);
    const vec3_t out = { r.x, r.y, r.z };
    return out;
}

/* 世界系向量转到机体系：v_b = R(q)ᵀ · v_w */
static inline vec3_t quat_rotate_inv(const quat_t *q, const vec3_t *v)
{
    const quat_t qc = quat_conj(q);
    return quat_rotate(&qc, v);
}

/* 由角速度与时间步长构造增量旋转，一阶近似 */
static inline quat_t quat_from_gyro(const vec3_t *omega, float dt)
{
    const float hx = 0.5f * omega->x * dt;
    const float hy = 0.5f * omega->y * dt;
    const float hz = 0.5f * omega->z * dt;
    const quat_t q = { 1.0f, hx, hy, hz };
    return quat_normalize(q);
}

/* 由两向量之间的最小旋转构造四元数，用于倾角与航向校正 */
static inline quat_t quat_from_two_vectors(const vec3_t *from, const vec3_t *to)
{
    const float dot = from->x * to->x + from->y * to->y + from->z * to->z;
    if (dot > 0.999999f) {
        return quat_identity();
    }
    if (dot < -0.999999f) {
        /* 反向，绕任意正交轴转 180 度 */
        vec3_t axis = { 1.0f, 0.0f, 0.0f };
        if (fabsf(from->y) < fabsf(from->x)) {
            axis.x = 0.0f;
            axis.y = 1.0f;
        }
        const quat_t q = { 0.0f, axis.x, axis.y, axis.z };
        return quat_normalize(q);
    }
    const vec3_t c = {
        from->y * to->z - from->z * to->y,
        from->z * to->x - from->x * to->z,
        from->x * to->y - from->y * to->x
    };
    const quat_t q = { 1.0f + dot, c.x, c.y, c.z };
    return quat_normalize(q);
}

/* 球面插值，t 在 0..1 */
static inline quat_t quat_slerp(const quat_t *a, const quat_t *b, float t)
{
    float dot = a->w * b->w + a->x * b->x + a->y * b->y + a->z * b->z;
    quat_t bb = *b;
    if (dot < 0.0f) {
        bb.w = -bb.w;
        bb.x = -bb.x;
        bb.y = -bb.y;
        bb.z = -bb.z;
        dot = -dot;
    }
    if (dot > 0.9995f) {
        const quat_t r = {
            a->w + t * (bb.w - a->w),
            a->x + t * (bb.x - a->x),
            a->y + t * (bb.y - a->y),
            a->z + t * (bb.z - a->z)
        };
        return quat_normalize(r);
    }
    const float theta = acosf(dot);
    const float s = sinf(theta);
    const float wa = sinf((1.0f - t) * theta) / s;
    const float wb = sinf(t * theta) / s;
    const quat_t r = {
        wa * a->w + wb * bb.w,
        wa * a->x + wb * bb.x,
        wa * a->y + wb * bb.y,
        wa * a->z + wb * bb.z
    };
    return quat_normalize(r);
}

static inline float vec3_norm(const vec3_t *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

static inline vec3_t vec3_sub(const vec3_t *a, const vec3_t *b)
{
    const vec3_t r = { a->x - b->x, a->y - b->y, a->z - b->z };
    return r;
}

static inline vec3_t vec3_add(const vec3_t *a, const vec3_t *b)
{
    const vec3_t r = { a->x + b->x, a->y + b->y, a->z + b->z };
    return r;
}

static inline vec3_t vec3_scale(const vec3_t *v, float s)
{
    const vec3_t r = { v->x * s, v->y * s, v->z * s };
    return r;
}

static inline vec3_t vec3_cross(const vec3_t *a, const vec3_t *b)
{
    const vec3_t r = {
        a->y * b->z - a->z * b->y,
        a->z * b->x - a->x * b->z,
        a->x * b->y - a->y * b->x
    };
    return r;
}
