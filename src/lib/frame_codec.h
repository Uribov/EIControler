#pragma once

/* 姿态帧的打包与校验。纯 C，不依赖 ESP-IDF，固件与 PC 测试共用同一份实现。 */

#include <stddef.h>
#include <stdint.h>

#include "common/types.h"

#define EI_FRAME_SIZE 210

uint16_t frame_crc16(const uint8_t *data, size_t len);

/* 打包一帧，返回字节数；容量不足返回 0 */
size_t frame_build(uint8_t *buf, size_t cap, const fusion_frame_t *f, uint16_t seq);
