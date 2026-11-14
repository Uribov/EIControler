#include "board.h"
#include "pinmap.h"

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_timer.h"

#define BOARD_SPI_HOST       SPI2_HOST
#define BOARD_SPI_CLK_HZ     (15 * 1000 * 1000)
#define BOARD_SPI_MAX_BYTES  4096

#define BOARD_I2C_PORT       I2C_NUM_0
#define BOARD_TIMER_PERIOD_US 5000

static spi_device_handle_t s_spi;
static i2c_port_t s_i2c = I2C_NUM_0;
static esp_timer_handle_t s_timer;
static void (*s_tick_cb)(void *arg);
static void *s_tick_arg;

static esp_err_t board_gpio_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_MUX_A0) |
                        (1ULL << PIN_MUX_A1) |
                        (1ULL << PIN_MUX_A2) |
                        (1ULL << PIN_MUX_A3) |
                        (1ULL << PIN_CS_E),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    (void)gpio_set_level(PIN_CS_E, 1);
    (void)gpio_set_level(PIN_MUX_A0, 0);
    (void)gpio_set_level(PIN_MUX_A1, 0);
    (void)gpio_set_level(PIN_MUX_A2, 0);
    (void)gpio_set_level(PIN_MUX_A3, 0);
    return ESP_OK;
}

static esp_err_t board_spi_init(void)
{
    const spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BOARD_SPI_MAX_BYTES,
    };
    const spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = BOARD_SPI_CLK_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 4,
    };

    esp_err_t err = spi_bus_initialize(BOARD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return err;
    }

    return spi_bus_add_device(BOARD_SPI_HOST, &dev_cfg, &s_spi);
}

static esp_err_t board_i2c_init(void)
{
    const i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    esp_err_t err = i2c_param_config(BOARD_I2C_PORT, &cfg);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(BOARD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t board_init(void)
{
    esp_err_t err = board_gpio_init();
    if (err != ESP_OK) {
        return err;
    }

    err = board_spi_init();
    if (err != ESP_OK) {
        return err;
    }

    return board_i2c_init();
}

void board_select(uint8_t idx)
{
    (void)gpio_set_level(PIN_CS_E, 1);
    (void)gpio_set_level(PIN_MUX_A0, (idx >> 0) & 0x01);
    (void)gpio_set_level(PIN_MUX_A1, (idx >> 1) & 0x01);
    (void)gpio_set_level(PIN_MUX_A2, (idx >> 2) & 0x01);
    (void)gpio_set_level(PIN_MUX_A3, (idx >> 3) & 0x01);
    (void)gpio_set_level(PIN_CS_E, 0);
}

void board_deselect(void)
{
    (void)gpio_set_level(PIN_CS_E, 1);
}

spi_device_handle_t board_spi(void)
{
    return s_spi;
}

i2c_port_t board_i2c(void)
{
    return s_i2c;
}

static void board_timer_trampoline(void *arg)
{
    (void)arg;
    if (s_tick_cb != NULL) {
        s_tick_cb(s_tick_arg);
    }
}

esp_err_t board_timer_start(void (*cb)(void *arg), void *arg)
{
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_timer != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_tick_cb = cb;
    s_tick_arg = arg;

    const esp_timer_create_args_t timer_cfg = {
        .callback = board_timer_trampoline,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "board_tick",
    };

    esp_err_t err = esp_timer_create(&timer_cfg, &s_timer);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_timer_start_periodic(s_timer, BOARD_TIMER_PERIOD_US);
    if (err != ESP_OK) {
        (void)esp_timer_delete(s_timer);
        s_timer = NULL;
    }
    return err;
}
