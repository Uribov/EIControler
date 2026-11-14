#include "sample.h"

#include <string.h>

#include "board/board.h"
#include "drivers/qmi8658a.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "sample";

#define SAMPLE_PERIOD_US 5000
#define QUEUE_DEPTH      4
#define SAMPLE_TASK_PRIO 18
#define SAMPLE_TASK_CORE 1
#define SAMPLE_STACK     4096
#define TEMP_PERIOD_MS   1000

static QueueHandle_t s_queue;
static TaskHandle_t s_task;
static sample_stats_t s_stats;
static float s_temps[EI_IMU_COUNT];

static void sample_task(void *arg);

/* 定时器回调只发通知，不做任何耗时操作 */
static void on_tick(void *arg)
{
    (void)arg;
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}

/* 解析 FIFO 出来的字节流：每样本 12 字节，加速度在前、陀螺在后，小端 */
static void parse_slot(imu_slot_t *slot, const uint8_t *raw, uint8_t count)
{
    slot->count = count;
    slot->valid = true;

    for (uint8_t k = 0; k < count; k++) {
        const uint8_t *p = raw + (size_t)k * 12u;
        for (int axis = 0; axis < 3; axis++) {
            const uint8_t *q = p + (size_t)axis * 2u;
            slot->acc[k][axis] = (int16_t)((uint16_t)q[0] | ((uint16_t)q[1] << 8));
        }
        for (int axis = 0; axis < 3; axis++) {
            const uint8_t *q = p + 6u + (size_t)axis * 2u;
            slot->gyro[k][axis] = (int16_t)((uint16_t)q[0] | ((uint16_t)q[1] << 8));
        }
    }
}

static void scan_once(raw_batch_t *batch)
{
    uint8_t raw[EI_MAX_BATCH_SAMP * 12];

    batch->ts_us = (uint32_t)esp_timer_get_time();

    for (uint8_t i = 0; i < EI_IMU_COUNT; i++) {
        imu_slot_t *slot = &batch->imu[i];
        slot->valid = false;
        slot->count = 0;

        uint8_t got = 0;
        const esp_err_t err = qmi_fifo_read(i, raw, EI_MAX_BATCH_SAMP, &got);

        if (err == ESP_ERR_TIMEOUT) {
            s_stats.comm_err++;
            continue;
        }
        if (err != ESP_OK) {
            s_stats.drop_imu++;
            continue;
        }
        if (got == 0) {
            continue; /* FIFO 里没数据属于正常 */
        }
        if (got >= EI_MAX_BATCH_SAMP) {
            s_stats.overrun++;
        }

        parse_slot(slot, raw, got);
        s_stats.samples_total += got;
    }
}

static void temp_task(void *arg)
{
    (void)arg;
    for (;;) {
        for (uint8_t i = 0; i < EI_IMU_COUNT; i++) {
            float t = 0.0f;
            if (qmi_read_temp(i, &t) == ESP_OK) {
                s_temps[i] = t;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TEMP_PERIOD_MS));
    }
}

static void sample_task(void *arg)
{
    (void)arg;
    raw_batch_t batch;

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 0) {
            continue; /* 定时器没来，继续等 */
        }

        scan_once(&batch);

        if (xQueueSend(s_queue, &batch, 0) != pdTRUE) {
            /* 队列满：丢最旧的一批，再塞新的 */
            raw_batch_t drop;
            (void)xQueueReceive(s_queue, &drop, 0);
            (void)xQueueSend(s_queue, &batch, 0);
            s_stats.drop_batch++;
        }
    }
}

esp_err_t sample_start(void)
{
    s_queue = xQueueCreate(QUEUE_DEPTH, sizeof(raw_batch_t));
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_stats, 0, sizeof(s_stats));

    if (xTaskCreatePinnedToCore(sample_task, "sample", SAMPLE_STACK, NULL,
                                SAMPLE_TASK_PRIO, &s_task, SAMPLE_TASK_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(temp_task, "temp", 3072, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t err = board_timer_start(on_tick, NULL);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "采样启动，周期 %d us", SAMPLE_PERIOD_US);
    return ESP_OK;
}

bool sample_fetch(raw_batch_t *out, uint32_t timeout_ms)
{
    if (out == NULL || s_queue == NULL) {
        return false;
    }
    return xQueueReceive(s_queue, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void sample_get_temps(float out[EI_IMU_COUNT])
{
    if (out != NULL) {
        memcpy(out, s_temps, sizeof(s_temps));
    }
}

sample_stats_t sample_get_stats(void)
{
    return s_stats;
}
