#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/types.h"
#include "esp_err.h"
#include "lib/frame_codec.h"

#define IO_FRAME_SIZE EI_FRAME_SIZE

typedef struct {
    uint32_t sent;
    uint32_t drop;
    uint32_t usb_err;
} io_stats_t;

esp_err_t io_start(void);

/* 纯打包，不发送，便于 PC 端与上位机对帧 */
size_t io_build_frame(uint8_t *buf, size_t cap, const fusion_frame_t *f, uint16_t seq);

io_stats_t io_get_stats(void);

/* CRC16-CCITT-FALSE，上位机用同一份实现校验 */
uint16_t io_crc16(const uint8_t *data, size_t len);
