/* EI Controller - 体感手套主控
 * 硬件: ESP32-S3 SuperMini + 74HC154 + 12x QMI8658A (U3-U14) + 1x QMC5883P (U15)
 *
 * SPI 引脚 (easyeda sch connectivity 核实):
 *   SCK=GP13  MOSI=GP12  MISO=GP11
 *   A0=GP8  A1=GP9  A2=GP10  A3=GP5   CS_E=GP4
 * I2C 引脚 (磁力计):
 *   SDA=GP7  SCL=GP6
 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board/board.h"
#include "common/types.h"
#include "config/config.h"
#include "drivers/qmi8658a.h"
#include "fusion/fusion.h"
#include "io/io.h"
#include "sample/sample.h"

static const char *TAG = "main";

static int init_imus(void)
{
    int ok = 0;
    for (uint8_t i = 0; i < EI_IMU_COUNT; i++) {
        const esp_err_t err = qmi_init(i);
        if (err == ESP_OK) {
            ok++;
        } else {
            ESP_LOGW(TAG, "IMU[%02u] 初始化失败: %s", (unsigned)i, esp_err_to_name(err));
        }
    }
    ESP_LOGI(TAG, "IMU 初始化完成 %d/%d", ok, EI_IMU_COUNT);
    return ok;
}

void app_main(void)
{
    esp_err_t err = cfg_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cfg_init 失败: %s", esp_err_to_name(err));
        return;
    }

    err = board_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "board_init 失败: %s", esp_err_to_name(err));
        return;
    }

    (void)init_imus();

    err = fusion_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fusion_init 失败: %s", esp_err_to_name(err));
        return;
    }

    err = sample_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sample_start 失败: %s", esp_err_to_name(err));
        return;
    }

    err = io_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "io_start 失败: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "EI Controller 启动完成，手别=%s",
             cfg_get_hand() == 0 ? "右手" : "左手");

    raw_batch_t batch;
    uint32_t last_stat = 0;

    for (;;) {
        if (sample_fetch(&batch, 1000)) {
            fusion_process_batch(&batch);
        }

        /* 每 5 秒打一次运行统计，便于看丢帧 */
        const uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (now - last_stat >= 5000u) {
            last_stat = now;
            const sample_stats_t ss = sample_get_stats();
            const io_stats_t is = io_get_stats();
            ESP_LOGI(TAG, "采样=%lu 丢批=%lu 丢颗=%lu 溢出=%lu | 发送=%lu 丢帧=%lu",
                     (unsigned long)ss.samples_total, (unsigned long)ss.drop_batch,
                     (unsigned long)ss.drop_imu, (unsigned long)ss.overrun,
                     (unsigned long)is.sent, (unsigned long)is.drop);
        }
    }
}
