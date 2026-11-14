#include "qmi8658a.h"
#include "../board/board.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define QMI_CHIP_COUNT        12

#define QMI_REG_WHO_AM_I      0x00
#define QMI_REG_CTRL1         0x02
#define QMI_REG_CTRL2         0x03
#define QMI_REG_CTRL3         0x04
#define QMI_REG_CTRL5         0x06
#define QMI_REG_CTRL7         0x08
#define QMI_REG_CTRL9         0x0A
#define QMI_REG_FIFO_WTM_TH   0x13
#define QMI_REG_FIFO_CTRL     0x14
#define QMI_REG_FIFO_SMPL_CNT 0x15
#define QMI_REG_FIFO_STATUS   0x16
#define QMI_REG_FIFO_DATA     0x17
#define QMI_REG_STATUSINT     0x2D
#define QMI_REG_TEMP_L        0x33
#define QMI_REG_TEMP_H        0x34

#define QMI_WHO_AM_I_VALUE    0x05
#define QMI_CTRL1_RESET       0x80
#define QMI_CTRL1_CONFIG      0x40
#define QMI_CTRL2_CONFIG      0x23
#define QMI_CTRL3_CONFIG      0x63
#define QMI_FIFO_CTRL_CONFIG  0x41
#define QMI_FIFO_READ_CMD     0x05

#define QMI_FIFO_STATUS_NONEMPTY 0x10
#define QMI_STATUSINT_RDY        0x80
#define QMI_FIFO_CTRL_RD_MODE    0x80
#define QMI_FIFO_SAMPLE_BYTES    12
#define QMI_FIFO_POLL_TIMEOUT_US 10000

static esp_err_t qmi_transfer(uint8_t cs_index, const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (cs_index >= QMI_CHIP_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_device_handle_t dev = board_spi();
    if (dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    board_select(cs_index);
    esp_err_t err = spi_device_polling_transmit(dev, &t);
    board_deselect();

    return (err == ESP_OK) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t qmi_read_burst(uint8_t cs_index, uint8_t reg, uint8_t *buf, size_t len)
{
    const size_t total = len + 2;
    uint8_t *tx = calloc(1, total);
    uint8_t *rx = malloc(total);

    if (tx == NULL || rx == NULL) {
        free(tx);
        free(rx);
        return ESP_ERR_NO_MEM;
    }

    tx[0] = (uint8_t)(0x80 | reg);
    esp_err_t err = qmi_transfer(cs_index, tx, rx, total);
    if (err == ESP_OK) {
        memcpy(buf, rx + 2, len);
    }

    free(tx);
    free(rx);
    return err;
}

esp_err_t qmi_read_reg(uint8_t cs_index, uint8_t reg, uint8_t *val)
{
    if (val == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t tx[3] = { (uint8_t)(0x80 | reg), 0x00, 0x00 };
    uint8_t rx[3] = { 0 };

    esp_err_t err = qmi_transfer(cs_index, tx, rx, sizeof(tx));
    if (err == ESP_OK) {
        *val = rx[2];
    }
    return err;
}

esp_err_t qmi_write_reg(uint8_t cs_index, uint8_t reg, uint8_t val)
{
    const uint8_t tx[2] = { reg, val };
    return qmi_transfer(cs_index, tx, NULL, sizeof(tx));
}

esp_err_t qmi_init(uint8_t cs_index)
{
    esp_err_t err = qmi_write_reg(cs_index, QMI_REG_CTRL1, QMI_CTRL1_RESET);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(15));

    uint8_t who = 0;
    err = qmi_read_reg(cs_index, QMI_REG_WHO_AM_I, &who);
    if (err != ESP_OK) {
        return err;
    }
    if (who != QMI_WHO_AM_I_VALUE) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = qmi_write_reg(cs_index, QMI_REG_CTRL1, QMI_CTRL1_CONFIG);
    if (err != ESP_OK) {
        return err;
    }
    err = qmi_write_reg(cs_index, QMI_REG_CTRL2, QMI_CTRL2_CONFIG);
    if (err != ESP_OK) {
        return err;
    }
    err = qmi_write_reg(cs_index, QMI_REG_CTRL3, QMI_CTRL3_CONFIG);
    if (err != ESP_OK) {
        return err;
    }
    err = qmi_write_reg(cs_index, QMI_REG_CTRL5, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    err = qmi_write_reg(cs_index, QMI_REG_CTRL7, 0x03);
    if (err != ESP_OK) {
        return err;
    }
    err = qmi_write_reg(cs_index, QMI_REG_FIFO_WTM_TH, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    return qmi_write_reg(cs_index, QMI_REG_FIFO_CTRL, QMI_FIFO_CTRL_CONFIG);
}

esp_err_t qmi_fifo_level(uint8_t cs_index, uint8_t *count)
{
    if (count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status = 0;
    esp_err_t err = qmi_read_reg(cs_index, QMI_REG_FIFO_STATUS, &status);
    if (err != ESP_OK) {
        return err;
    }

    if ((status & QMI_FIFO_STATUS_NONEMPTY) == 0) {
        *count = 0;
        return ESP_OK;
    }

    uint8_t bytes = 0;
    err = qmi_read_reg(cs_index, QMI_REG_FIFO_SMPL_CNT, &bytes);
    if (err != ESP_OK) {
        return err;
    }

    *count = bytes / QMI_FIFO_SAMPLE_BYTES;
    return ESP_OK;
}

esp_err_t qmi_fifo_read(uint8_t cs_index, uint8_t *buf, uint8_t max_samples, uint8_t *got)
{
    if (buf == NULL || got == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *got = 0;

    uint8_t level = 0;
    esp_err_t err = qmi_fifo_level(cs_index, &level);
    if (err != ESP_OK) {
        return err;
    }
    if (level > max_samples) {
        level = max_samples;
    }
    if (level == 0) {
        return ESP_OK;
    }

    err = qmi_write_reg(cs_index, QMI_REG_CTRL9, QMI_FIFO_READ_CMD);
    if (err != ESP_OK) {
        return err;
    }

    const int64_t deadline = esp_timer_get_time() + QMI_FIFO_POLL_TIMEOUT_US;
    for (;;) {
        uint8_t status = 0;
        err = qmi_read_reg(cs_index, QMI_REG_STATUSINT, &status);
        if (err != ESP_OK) {
            (void)qmi_write_reg(cs_index, QMI_REG_CTRL9, 0x00);
            return err;
        }
        if ((status & QMI_STATUSINT_RDY) != 0) {
            break;
        }
        if (esp_timer_get_time() >= deadline) {
            (void)qmi_write_reg(cs_index, QMI_REG_CTRL9, 0x00);
            return ESP_ERR_TIMEOUT;
        }
    }

    err = qmi_read_burst(cs_index, QMI_REG_FIFO_DATA,
                         buf, (size_t)level * QMI_FIFO_SAMPLE_BYTES);
    if (err == ESP_OK) {
        *got = level;
    }

    esp_err_t ack_err = qmi_write_reg(cs_index, QMI_REG_CTRL9, 0x00);
    if (err == ESP_OK) {
        err = ack_err;
    }

    uint8_t fifo_ctrl = 0;
    if (qmi_read_reg(cs_index, QMI_REG_FIFO_CTRL, &fifo_ctrl) == ESP_OK) {
        (void)qmi_write_reg(cs_index, QMI_REG_FIFO_CTRL,
                            (uint8_t)(fifo_ctrl & ~QMI_FIFO_CTRL_RD_MODE));
    }

    return err;
}

esp_err_t qmi_read_temp(uint8_t cs_index, float *temp_c)
{
    if (temp_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t lo = 0;
    uint8_t hi = 0;

    esp_err_t err = qmi_read_reg(cs_index, QMI_REG_TEMP_L, &lo);
    if (err != ESP_OK) {
        return err;
    }
    err = qmi_read_reg(cs_index, QMI_REG_TEMP_H, &hi);
    if (err != ESP_OK) {
        return err;
    }

    const int16_t raw = (int16_t)(((uint16_t)hi << 8) | lo);
    *temp_c = (float)raw / 256.0f;
    return ESP_OK;
}
