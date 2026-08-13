/*
 * Hitachi AC MITM - status_led.cpp
 *
 * WS2812B 狀態指示燈 driver（GPIO8，沿用 ESP32-C6 DevKitC-1 板載 LED）
 *
 * 職責：
 *   - 用 led_strip RMT 驅動單顆 WS2812B（佔 RMT TX channel 1，1 block = 48 symbols）
 *   - 依系統狀態顯示不同顏色/閃爍：
 *       OFF          → 熄燈
 *       COMMISSIONING→ 藍色閃爍（配對中）
 *       CONNECTED    → 綠色（已加入 fabric）
 *       AC_ON        → 紅色（冷氣運轉中）
 *       IDENTIFY     → 黃色閃爍（Matter identify）
 *   - 背景 task 處理閃爍動畫（500ms 週期）
 *
 * RMT 通道使用狀況（ESP32-C6 共 4 通道：2 TX + 2 RX）：
 *   TX channel 0 → IR 發射（ir_driver.cpp，48 symbols）
 *   TX channel 1 → WS2812B LED（本 driver，48 symbols）
 *   RX channel 2,3 → IR 接收（ir_driver.cpp，96 symbols ping-pong）
 */

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "led_strip.h"

#include "app_priv.h"

static const char *TAG = "status_led";

/* ========== 全域狀態 ========== */
static led_strip_handle_t s_strip = NULL;
static status_led_state_t s_current_state = STATUS_LED_OFF;
static SemaphoreHandle_t s_state_mutex = NULL;   /* 保護 s_current_state */
static TaskHandle_t s_blink_task = NULL;
static bool s_initialized = false;

/* ========== 顏色定義（RGB，0-255） ========== */
typedef struct {
    uint8_t r, g, b;
} rgb_t;

static const rgb_t COLOR_OFF          = {0,   0,   0};
static const rgb_t COLOR_BLUE         = {0,   0,   255};   /* 配對中 */
static const rgb_t COLOR_GREEN        = {0,   255, 0};     /* 已連線 */
static const rgb_t COLOR_RED          = {255, 0,   0};     /* 冷氣運轉 */
static const rgb_t COLOR_YELLOW       = {255, 200, 0};     /* 識別中 */
static const rgb_t COLOR_PURPLE       = {255, 0,   255};   /* 燒錄完成/啟動中 */

/* ========== 內部：設定 LED 顏色並刷新 ========== */
static void status_led_set_rgb(const rgb_t *color)
{
    if (s_strip == NULL) {
        return;
    }
    led_strip_set_pixel(s_strip, 0, color->r, color->g, color->b);
    led_strip_refresh(s_strip);
}

/* ========== 背景閃爍 task ========== */
static void status_led_blink_task(void *arg)
{
    static const TickType_t k_blink_period = pdMS_TO_TICKS(500);  /* 500ms 週期 */
    bool on_phase = true;

    while (true) {
        status_led_state_t state;
        if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            state = s_current_state;
            xSemaphoreGive(s_state_mutex);
        } else {
            state = STATUS_LED_OFF;
        }

        switch (state) {
        case STATUS_LED_OFF:
            status_led_set_rgb(&COLOR_OFF);
            vTaskDelay(k_blink_period);
            break;

        case STATUS_LED_BOOT:
            /* 紫色恆亮：燒錄完成/啟動中 */
            status_led_set_rgb(&COLOR_PURPLE);
            vTaskDelay(k_blink_period);
            break;

        case STATUS_LED_COMMISSIONING:
            /* 藍色閃爍：on 200ms / off 300ms */
            status_led_set_rgb(on_phase ? &COLOR_BLUE : &COLOR_OFF);
            vTaskDelay(on_phase ? pdMS_TO_TICKS(200) : pdMS_TO_TICKS(300));
            on_phase = !on_phase;
            break;

        case STATUS_LED_IDENTIFY:
            /* 黃色快速閃爍：on 250ms / off 250ms */
            status_led_set_rgb(on_phase ? &COLOR_YELLOW : &COLOR_OFF);
            vTaskDelay(pdMS_TO_TICKS(250));
            on_phase = !on_phase;
            break;

        case STATUS_LED_CONNECTED:
            /* 綠色恆亮 */
            status_led_set_rgb(&COLOR_GREEN);
            vTaskDelay(k_blink_period);
            break;

        case STATUS_LED_AC_ON:
            /* 紅色恆亮 */
            status_led_set_rgb(&COLOR_RED);
            vTaskDelay(k_blink_period);
            break;

        default:
            status_led_set_rgb(&COLOR_OFF);
            vTaskDelay(k_blink_period);
            break;
        }
    }
}

/* ========== 公開 API ========== */

esp_err_t status_led_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    /* led_strip 設定：單顆 WS2812B on GPIO8 */
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812B_LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = 0,
        },
    };

    /* RMT 設定：10MHz 解析度，48 symbols（1 block，佔 TX channel 1）
     * ESP32-C6 每通道 48 words，48 symbols 剛好 1 block，不會與 IR TX（channel 0）衝突 */
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        /* 預設時脈源 */
        .resolution_hz = 10 * 1000 * 1000,     /* 10MHz */
        .mem_block_symbols = 48,               /* 1 block，符合 ESP32-C6 單通道容量 */
        .flags = {
            .with_dma = false,                 /* ESP32-C6 RMT 無 DMA */
        },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    if (err != ESP_OK || s_strip == NULL) {
        ESP_LOGE(TAG, "WS2812B led_strip install failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    /* 建立 mutex 保護狀態 */
    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        led_strip_del(s_strip);
        s_strip = NULL;
        return ESP_FAIL;
    }

    /* 建立閃爍 task（低優先序，2KB stack） */
    if (xTaskCreate(status_led_blink_task, "status_led", 2048, NULL, 1, &s_blink_task) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create blink task");
        vSemaphoreDelete(s_state_mutex);
        led_strip_del(s_strip);
        s_strip = NULL;
        s_state_mutex = NULL;
        return ESP_FAIL;
    }

    /* 開機預設：燒錄完成/啟動中（紫色） */
    s_current_state = STATUS_LED_BOOT;
    s_initialized = true;

    ESP_LOGI(TAG, "WS2812B status LED ready on GPIO%d (RMT TX ch1, 48 symbols)", WS2812B_LED_GPIO);
    return ESP_OK;
}

void status_led_set_state(status_led_state_t state)
{
    if (!s_initialized || s_state_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_current_state != state) {
            ESP_LOGI(TAG, "State -> %d", (int)state);
            s_current_state = state;
        }
        xSemaphoreGive(s_state_mutex);
    }
}

void status_led_identify(bool start)
{
    status_led_set_state(start ? STATUS_LED_IDENTIFY : STATUS_LED_CONNECTED);
}
