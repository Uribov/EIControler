#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t board_init(void);
void board_select(uint8_t idx);
void board_deselect(void);
spi_device_handle_t board_spi(void);
i2c_port_t board_i2c(void);
esp_err_t board_timer_start(void (*cb)(void *arg), void *arg);

#ifdef __cplusplus
}
#endif
