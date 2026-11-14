#include "config.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "config";

#define CFG_NAMESPACE "eicfg"
#define CFG_VERSION   1u

#define KEY_HAND   "hand"
#define KEY_MAGCAL "magcal"
#define KEY_BIAS   "bias"
#define KEY_VER    "ver"
#define KEY_CALTS  "calts"

typedef struct {
    uint8_t hand;                 /* 0 右手，1 左手 */
    float   mag_cal[9];           /* 椭球系数，默认单位阵 */
    float   gyro_bias[CFG_IMU_COUNT][3];
    uint32_t cal_ts;
} cfg_data_t;

static cfg_data_t s_cfg;
static bool s_ready;

static void cfg_defaults(cfg_data_t *c)
{
    memset(c, 0, sizeof(*c));
    c->hand = 0;
    /* 单位阵：不做椭球校正 */
    c->mag_cal[0] = 1.0f;
    c->mag_cal[4] = 1.0f;
    c->mag_cal[8] = 1.0f;
}

static esp_err_t cfg_load(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(CFG_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        return err;
    }

    uint32_t ver = 0;
    err = nvs_get_u32(h, KEY_VER, &ver);
    if (err != ESP_OK || ver != CFG_VERSION) {
        nvs_close(h);
        return ESP_ERR_INVALID_VERSION;
    }

    size_t len = sizeof(s_cfg.gyro_bias);
    (void)nvs_get_blob(h, KEY_BIAS, s_cfg.gyro_bias, &len);

    len = sizeof(s_cfg.mag_cal);
    (void)nvs_get_blob(h, KEY_MAGCAL, s_cfg.mag_cal, &len);

    (void)nvs_get_u8(h, KEY_HAND, &s_cfg.hand);
    (void)nvs_get_u32(h, KEY_CALTS, &s_cfg.cal_ts);

    nvs_close(h);
    return ESP_OK;
}

esp_err_t cfg_init(void)
{
    cfg_defaults(&s_cfg);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        (void)nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    err = cfg_load();
    if (err == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "配置版本不符，清空重建");
        (void)cfg_erase_all();
        cfg_defaults(&s_cfg);
    } else if (err != ESP_OK) {
        ESP_LOGI(TAG, "无历史配置，使用默认值");
        cfg_defaults(&s_cfg);
    }

    s_ready = true;
    return ESP_OK;
}

uint8_t cfg_get_hand(void)
{
    return s_cfg.hand;
}

esp_err_t cfg_set_hand(uint8_t hand)
{
    if (hand > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg.hand = hand;
    return ESP_OK;
}

esp_err_t cfg_get_gyro_bias(uint8_t idx, float out[3])
{
    if (idx >= CFG_IMU_COUNT || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(out, s_cfg.gyro_bias[idx], sizeof(float) * 3);
    return ESP_OK;
}

esp_err_t cfg_set_gyro_bias(uint8_t idx, const float in[3])
{
    if (idx >= CFG_IMU_COUNT || in == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(s_cfg.gyro_bias[idx], in, sizeof(float) * 3);
    return ESP_OK;
}

esp_err_t cfg_get_mag_cal(float coef[9])
{
    if (coef == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(coef, s_cfg.mag_cal, sizeof(s_cfg.mag_cal));
    return ESP_OK;
}

esp_err_t cfg_set_mag_cal(const float coef[9])
{
    if (coef == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(s_cfg.mag_cal, coef, sizeof(s_cfg.mag_cal));
    return ESP_OK;
}

esp_err_t cfg_erase_all(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(CFG_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    (void)nvs_erase_all(h);
    nvs_close(h);
    s_cfg.cal_ts = 0;
    return ESP_OK;
}

esp_err_t cfg_save_all(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(CFG_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    s_cfg.cal_ts = (uint32_t)(esp_log_timestamp() / 1000u);

    err = nvs_set_u32(h, KEY_VER, CFG_VERSION);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, KEY_BIAS, s_cfg.gyro_bias, sizeof(s_cfg.gyro_bias));
    }
    if (err == ESP_OK) {
        err = nvs_set_blob(h, KEY_MAGCAL, s_cfg.mag_cal, sizeof(s_cfg.mag_cal));
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(h, KEY_HAND, s_cfg.hand);
    }
    if (err == ESP_OK) {
        err = nvs_set_u32(h, KEY_CALTS, s_cfg.cal_ts);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }

    nvs_close(h);
    return err;
}
