/*
 * Hitachi AC MITM - ds18b20_driver.cpp
 *
 * DS18B20 1-Wire 室溫感測器 driver（GPIO bit-bang 實作，不依賴 RMT）
 *
 * 接線：VCC→3.3V, GND→GND, DATA→GPIO1（需 4.7kΩ 外部上拉到 3.3V）
 *
 * 使用 GPIO bit-bang 實作 1-Wire 協定，避免與 IR driver 爭搶 RMT 通道。
 * 啟動一個 task 每 2 秒讀一次溫度，透過 callback 推到 Matter attribute
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "app_priv.h"

static const char *TAG = "ds18b20";

/* DS18B20 ROM commands */
#define DS18B20_CMD_SKIP_ROM        0xCC
#define DS18B20_CMD_CONVERT_T       0x44
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE

/* 12-bit 轉換最大 750ms */
#define DS18B20_CONVERT_MS  750

static TaskHandle_t s_task = NULL;
static float s_last_temp_c = DEFAULT_LOCAL_TEMP_C;
static float s_last_reported_temp_c = DEFAULT_LOCAL_TEMP_C;
static bool s_reported_once = false;
static bool s_device_present = false;

/* 溫度更新 callback（由 app_driver 註冊） */
typedef void (*temp_update_cb_t)(float temp_c);
static temp_update_cb_t s_update_cb = NULL;

/* ========== 1-Wire bit-bang 基礎常式 ==========
 * 1-Wire 匯流排為開漏極（open-drain）。ESP32 GPIO 設為 open-drain 輸出模式，
 * 只用 gpio_set_level() 切換（單一暫存器寫入，奈秒級），避免 gpio_set_direction()
 * 等慢速 HAL 呼叫破壞 1-Wire 微秒級時序。
 *   - gpio_set_level(pin, 0) → 拉低匯流排
 *   - gpio_set_level(pin, 1) → 釋放匯流排（high-Z，外部上拉拉高）
 * 讀取時 open-drain 輸出 1 仍可用 gpio_get_level() 讀到真實腳位電位。
 * ESP32-C6 @ 160MHz，esp_timer_get_time() 解析度 1us。
 */

static inline void ow_release_bus(void)
{
    gpio_set_level(DS18B20_GPIO, 1);  /* open-drain high-Z，上拉拉高 */
}

static inline void ow_pull_low(void)
{
    gpio_set_level(DS18B20_GPIO, 0);  /* open-drain 輸出 0，拉低 */
}

static inline int ow_read_bit(void)
{
    return gpio_get_level(DS18B20_GPIO) ? 1 : 0;
}

/* 精確微秒延遲（busy-wait，1-Wire timing 需要精確） */
static inline void ow_delay_us(uint32_t us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        /* busy wait */
    }
}

/* ========== 1-Wire 協定常式 ========== */

/* Reset pulse + presence detect
 * 回傳 true 若偵測到 DS18B20 presence pulse
 */
static bool ow_reset(void)
{
    bool present;

    /* 拉低 480us 以上（reset pulse） */
    ow_pull_low();
    ow_delay_us(500);

    /* 釋放匯流排，等待 70us 後取樣 presence */
    ow_release_bus();
    ow_delay_us(70);

    /* DS18B20 會拉低匯流排 60-240us 表示 presence */
    present = (ow_read_bit() == 0);

    /* 等待剩餘 presence pulse 結束（總共至少 480us） */
    ow_delay_us(430);

    return present;
}

/* 寫一個 bit（1 或 0） */
static void ow_write_bit(int bit)
{
    if (bit) {
        /* Write 1: 拉低 1-15us，釋放後 DS18B20 在 15-60us 內取樣 */
        ow_pull_low();
        ow_delay_us(6);
        ow_release_bus();
        ow_delay_us(64);
    } else {
        /* Write 0: 拉低 60-120us，DS18B20 在 15-60us 內取樣 */
        ow_pull_low();
        ow_delay_us(60);
        ow_release_bus();
        ow_delay_us(10);
    }
}

/* 讀一個 bit */
static int ow_read_bit_raw(void)
{
    int bit;

    /* Read slot: 拉低 1-15us，釋放後在 15us 內取樣 */
    ow_pull_low();
    ow_delay_us(6);
    ow_release_bus();
    ow_delay_us(9);
    bit = ow_read_bit();
    /* 等待 read slot 結束（總共至少 60us） */
    ow_delay_us(55);

    return bit;
}

/* 寫一個 byte（LSB first） */
static void ow_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        ow_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

/* 讀一個 byte（LSB first） */
static uint8_t ow_read_byte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (ow_read_bit_raw() << i);
    }
    return byte;
}

/* ========== DS18B20 高階操作 ========== */

static uint8_t ds18b20_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            byte >>= 1;
        }
    }
    return crc;
}

static bool ds18b20_scratchpad_valid(const uint8_t *scratchpad)
{
    return ds18b20_crc8(scratchpad, 8) == scratchpad[8];
}

#define DS18B20_READ_RETRY      3
#define DS18B20_RETRY_DELAY_MS  50

static esp_err_t ds18b20_read_raw(int16_t *raw)
{
    for (int attempt = 0; attempt <= DS18B20_READ_RETRY; attempt++) {
        if (!ow_reset()) {
            if (attempt < DS18B20_READ_RETRY) {
                vTaskDelay(pdMS_TO_TICKS(DS18B20_RETRY_DELAY_MS));
                continue;
            }
            return ESP_ERR_NOT_FOUND;
        }
        ow_write_byte(DS18B20_CMD_SKIP_ROM);
        ow_write_byte(DS18B20_CMD_CONVERT_T);

        vTaskDelay(pdMS_TO_TICKS(DS18B20_CONVERT_MS));

        if (!ow_reset()) {
            if (attempt < DS18B20_READ_RETRY) {
                vTaskDelay(pdMS_TO_TICKS(DS18B20_RETRY_DELAY_MS));
                continue;
            }
            return ESP_ERR_NOT_FOUND;
        }
        ow_write_byte(DS18B20_CMD_SKIP_ROM);
        ow_write_byte(DS18B20_CMD_READ_SCRATCHPAD);

        uint8_t scratchpad[9];
        for (int i = 0; i < 9; i++) {
            scratchpad[i] = ow_read_byte();
        }

        if (ds18b20_scratchpad_valid(scratchpad)) {
            *raw = (int16_t)((scratchpad[1] << 8) | scratchpad[0]);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "CRC mismatch: calc=0x%02X recv=0x%02X (attempt %d/%d)",
                 ds18b20_crc8(scratchpad, 8), scratchpad[8], attempt + 1, DS18B20_READ_RETRY + 1);
        if (attempt < DS18B20_READ_RETRY) {
            vTaskDelay(pdMS_TO_TICKS(DS18B20_RETRY_DELAY_MS));
        }
    }
    return ESP_ERR_INVALID_CRC;
}

static void ds18b20_task(void *arg)
{
    int fail_count = 0;
    /* 首次延遲 1 秒，讓其他 driver 穩定 */
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (true) {
        int16_t raw;
        esp_err_t err = ds18b20_read_raw(&raw);
        if (err == ESP_OK) {
            float temp_c = raw / 16.0f;
            s_last_temp_c = temp_c;
            s_device_present = true;
            ESP_LOGI(TAG, "Temperature: %.2f°C (raw=0x%04x)", temp_c, (uint16_t)raw);

            /* 只有在差異 >= 0.5°C 或首次讀取才推給 Matter，避免頻繁更新 attribute。
             * 注意：s_reported_once / s_last_reported_temp_c 只在 callback 已註冊時
             * 才設置，否則 ds18b20_task 在 app_driver_register_callbacks() 之前讀到
             * 首次溫度會把 s_reported_once=true，導致 callback 註冊後因溫度穩定
             * (diff < 0.5°C) 永遠不再報告（race condition）。 */
            float diff = temp_c - s_last_reported_temp_c;
            if (diff < 0) diff = -diff;
            if (!s_reported_once || diff >= 0.5f) {
                if (s_update_cb) {
                    s_last_reported_temp_c = temp_c;
                    s_reported_once = true;
                    s_update_cb(temp_c);
                }
            }
            fail_count = 0;
        } else {
            fail_count++;
            s_device_present = false;
            if (fail_count <= 3 || fail_count % 10 == 0) {
                ESP_LOGW(TAG, "ds18b20_read_raw failed: %s (fail=%d)",
                         esp_err_to_name(err), fail_count);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(DS18B20_READ_INTERVAL_MS));
    }
}

esp_err_t ds18b20_driver_init(void)
{
    esp_err_t err;

    /* 設定 GPIO 為 input + open-drain 輸出。
     * 注意：不可用 GPIO_MODE_OUTPUT_OD，它會關閉 input buffer 導致 gpio_get_level()
     * 永遠讀到 0。必須用 GPIO_MODE_INPUT_OUTPUT_OD 同時啟用 input + OD output。
     * 釋放匯流排 = gpio_set_level(1)（high-Z，外部 4.7kΩ 上拉拉高）。 */
    err = gpio_set_direction(DS18B20_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_set_direction failed: %s", esp_err_to_name(err));
        return err;
    }
    gpio_set_level(DS18B20_GPIO, 1);  /* 釋放匯流排 */

    /* 偵測 DS18B20 是否存在 */
    bool present = ow_reset();
    if (present) {
        ESP_LOGI(TAG, "DS18B20 detected on GPIO%d", DS18B20_GPIO);
        s_device_present = true;
    } else {
        ESP_LOGW(TAG, "No DS18B20 detected on GPIO%d (will retry in task)", DS18B20_GPIO);
        s_device_present = false;
    }

    /* 啟動讀取 task */
    BaseType_t ok = xTaskCreate(ds18b20_task, "ds18b20", 4096, NULL, 5, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate ds18b20_task failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "DS18B20 driver ready on GPIO%d (bit-bang mode)", DS18B20_GPIO);
    return ESP_OK;
}

void ds18b20_register_update_cb(void (*cb)(float temp_c))
{
    s_update_cb = cb;
}

float ds18b20_get_last_temp(void)
{
    return s_last_temp_c;
}

bool ds18b20_is_present(void)
{
    return s_device_present;
}
