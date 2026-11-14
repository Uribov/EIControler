#pragma once

#include "esp_err.h"
#include "lib/quat.h"

/* QMC5883P 三轴磁力计，I2C 接口，地址 0x2C */

esp_err_t qmc_init(void);

/* 读一次磁场，单位微特斯拉。无新数据时返回 ESP_ERR_NOT_FOUND */
esp_err_t qmc_read(vec3_t *mag_ut);

esp_err_t qmc_read_chip_id(uint8_t *id);
