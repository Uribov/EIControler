#include "qmc5883p.h"

#include <string.h>

#include "board/board.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "qmc5883p";

#define QMC_ADDR        0x2C
#define QMC_I2C_TIMEOUT pdMS_TO_TICKS(20)

#define QMC_REG_CHIP_ID 0x00
#define QMC_REG_X_LSB   0x01
#define QMC_REG_STATUS  0x09
#define QMC_REG_CNTL1   0x0A
#define QMC_REG_CNTL2   0x0B

#define QMC_CHIP_ID_VAL 0x80

/* CNTL1: 连续模式 + 8 倍过采样 + 50Hz */
#define QMC_CNTL1_VAL   0x37
/* CNTL2: ±2G 量程，关自检与复位 */
#define QMC_CNTL2_VAL   0x0C

#define QMC_STATUS_DRDY 0x01

/* ±2G 量程下 15000 LSB/Gauss（手册 Table 2），1G = 100uT，所以 150 LSB/uT */
#define QMC_LSB_PER_UT  150.0f

static esp_err_t qmc_write(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(board_i2c(), QMC_ADDR, buf, sizeof(buf),
                                      QMC_I2C_TIMEOUT);
}

static esp_err_t qmc_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(board_i2c(), QMC_ADDR, &reg, 1, buf, len,
                                        QMC_I2C_TIMEOUT);
}

esp_err_t qmc_read_chip_id(uint8_t *id)
{
    if (id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return qmc_read_bytes(QMC_REG_CHIP_ID, id, 1);
}

esp_err_t qmc_init(void)
{
    uint8_t id = 0;
    esp_err_t err = qmc_read_chip_id(&id);
    if (err != ESP_OK) {
        return err;
    }
    if (id != QMC_CHIP_ID_VAL) {
        ESP_LOGW(TAG, "CHIP_ID=0x%02X，期望 0x80", id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = qmc_write(QMC_REG_CNTL2, QMC_CNTL2_VAL);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    err = qmc_write(QMC_REG_CNTL1, QMC_CNTL1_VAL);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "初始化完成");
    return ESP_OK;
}

esp_err_t qmc_read(vec3_t *mag_ut)
{
    if (mag_ut == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status = 0;
    esp_err_t err = qmc_read_bytes(QMC_REG_STATUS, &status, 1);
    if (err != ESP_OK) {
        return err;
    }
    if ((status & QMC_STATUS_DRDY) == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t raw[6];
    err = qmc_read_bytes(QMC_REG_X_LSB, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    const int16_t x = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    const int16_t y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    const int16_t z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

    /* 实测朝向（以人体自然坐标系为准）：
     *   磁力计 x+ 朝右、y+ 朝前、z+ 朝上
     *   IMU     x+ 朝后、y+ 朝右、z+ 朝上
     * 磁力计的 x 对应 IMU 的 +y，磁力计的 y（前）对应 IMU 的 -x。
     * 所以 x 取负的 my，y 取 mx，z 不变。 */
    const float mx = (float)x / QMC_LSB_PER_UT;
    const float my = (float)y / QMC_LSB_PER_UT;
    const float mz = (float)z / QMC_LSB_PER_UT;

    mag_ut->x = -my;
    mag_ut->y = mx;
    mag_ut->z = mz;
    return ESP_OK;
}
