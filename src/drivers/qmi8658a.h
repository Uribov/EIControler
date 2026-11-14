#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t qmi_init(uint8_t cs_index);
esp_err_t qmi_fifo_level(uint8_t cs_index, uint8_t *count);
esp_err_t qmi_fifo_read(uint8_t cs_index, uint8_t *buf, uint8_t max_samples, uint8_t *got);
esp_err_t qmi_read_temp(uint8_t cs_index, float *temp_c);
esp_err_t qmi_read_reg(uint8_t cs_index, uint8_t reg, uint8_t *val);
esp_err_t qmi_write_reg(uint8_t cs_index, uint8_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif
