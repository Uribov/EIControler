#include "io.h"

#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fusion/fusion.h"
#include "lib/frame_codec.h"
#include "sample/sample.h"

static const char *TAG = "io";

#define IO_PERIOD_MS  5
#define IO_TASK_PRIO  10
#define IO_TASK_CORE  0
#define IO_TASK_STACK 4096

static io_stats_t s_stats;
static uint16_t s_seq;

uint16_t io_crc16(const uint8_t *data, size_t len)
{
    return frame_crc16(data, len);
}

size_t io_build_frame(uint8_t *buf, size_t cap, const fusion_frame_t *f, uint16_t seq)
{
    return frame_build(buf, cap, f, seq);
}

static void io_task(void *arg)
{
    (void)arg;
    fusion_frame_t frame;
    float temps[EI_IMU_COUNT];
    uint8_t buf[EI_FRAME_SIZE];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(IO_PERIOD_MS));

        sample_get_temps(temps);
        fusion_get_frame(&frame, temps);

        const size_t n = frame_build(buf, sizeof(buf), &frame, s_seq);
        if (n == 0) {
            continue;
        }
        s_seq++;

        const int written = usb_serial_jtag_write_bytes(buf, n, 0);
        if (written == (int)n) {
            s_stats.sent++;
        } else if (written < 0) {
            s_stats.usb_err++;
        } else {
            s_stats.drop++;
        }
    }
}

esp_err_t io_start(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 256,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    if (xTaskCreatePinnedToCore(io_task, "io", IO_TASK_STACK, NULL, IO_TASK_PRIO, NULL,
                                IO_TASK_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "发送任务启动，帧长 %d", EI_FRAME_SIZE);
    return ESP_OK;
}

io_stats_t io_get_stats(void)
{
    return s_stats;
}
