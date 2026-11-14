#include "frame_codec.h"

#include <string.h>

#define HDR0 0xAAu
#define HDR1 0x55u

uint16_t frame_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void put_f32(uint8_t *p, float f)
{
    uint32_t v;
    memcpy(&v, &f, sizeof(v));
    put_u32(p, v);
}

size_t frame_build(uint8_t *buf, size_t cap, const fusion_frame_t *f, uint16_t seq)
{
    if (buf == NULL || f == NULL || cap < EI_FRAME_SIZE) {
        return 0;
    }

    size_t o = 0;
    buf[o++] = HDR0;
    buf[o++] = HDR1;
    put_u16(&buf[o], seq);
    o += 2;
    put_u32(&buf[o], f->ts_ms);
    o += 4;

    for (int i = 0; i < 4; i++) {
        put_f32(&buf[o], f->q_hand[i]);
        o += 4;
    }

    for (int i = 0; i < EI_FINGER_COUNT; i++) {
        for (int k = 0; k < 4; k++) {
            put_f32(&buf[o], f->q_rel[i][k]);
            o += 4;
        }
    }

    for (int i = 0; i < EI_IMU_COUNT; i++) {
        int32_t t = (int32_t)(f->temp[i] * 100.0f);
        if (t > 32767) {
            t = 32767;
        } else if (t < -32768) {
            t = -32768;
        }
        put_u16(&buf[o], (uint16_t)(int16_t)t);
        o += 2;
    }

    put_u16(&buf[o], frame_crc16(buf, o));
    o += 2;

    return o;
}
