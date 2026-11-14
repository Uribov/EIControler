#pragma once

#include <stdint.h>

#include "esp_err.h"

#define CFG_IMU_COUNT 12

esp_err_t cfg_init(void);

uint8_t cfg_get_hand(void);
esp_err_t cfg_set_hand(uint8_t hand);

esp_err_t cfg_get_gyro_bias(uint8_t idx, float out[3]);
esp_err_t cfg_set_gyro_bias(uint8_t idx, const float in[3]);

esp_err_t cfg_get_mag_cal(float coef[9]);
esp_err_t cfg_set_mag_cal(const float coef[9]);

esp_err_t cfg_save_all(void);
esp_err_t cfg_erase_all(void);
