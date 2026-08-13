/*
 * Hitachi AC MITM - gpio_monitor.cpp
 *
 * GPIO 監控 driver，移植自 ESPHome binary_sensor (gpio platform)
 *
 * GPIO10 ← 冷氣面板指示燈 LED 光敏電阻 (LDR 分壓) = 冷氣電源狀態
 *   - LED 亮 = 冷氣運轉中 → LDR 低阻 → GPIO LOW (active-low)
 *   - LED 暗 = 冷氣關機   → LDR 高阻 → GPIO HIGH（內部 pull-up）
 *   - delayed_on: 500ms, delayed_off: 3s（移植自 ESPHome filters）
 * GPIO11 ← 220V 壓縮機光耦 = 壓縮機運轉狀態
 *   - delayed_on: 500ms, delayed_off: 500ms
 *
 * 用 polling task（每 100ms 讀一次）+ 軟體去抖動，避免 ISR 複雜度
 * 狀態變化時透過 callback 推到 Matter contact_sensor endpoint
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app_priv.h"

static const char *TAG = "gpio_mon";

/* 去抖動參數（移植自 ESPHome filters） */
#define POLL_INTERVAL_MS        100
#define AC_POWER_DELAYED_ON_MS   500   /* LED 亮→LDR 低阻，500ms 確認 */
#define AC_POWER_DELAYED_OFF_MS  3000  /* LED 暗→LDR 高阻，3s 確認（避免壓縮機暫停誤判） */
#define COMPRESSOR_DELAYED_ON_MS 500
#define COMPRESSOR_DELAYED_OFF_MS 500

/* callback（由 app_driver 註冊） */
typedef void (*gpio_state_cb_t)(bool state);
static gpio_state_cb_t s_ac_power_cb = NULL;
static gpio_state_cb_t s_compressor_cb = NULL;

/* 去抖動狀態 */
typedef struct {
    bool raw_level;          /* 最近一次讀到的 raw GPIO level */
    bool stable_state;       /* 已穩定的狀態 */
    int64_t change_time_us;  /* 上次 raw level 變化的時間 */
    uint32_t on_delay_ms;
    uint32_t off_delay_ms;
    gpio_num_t gpio;
    const char *name;
} debounce_t;

static debounce_t s_ac_power = {
    .stable_state = false,
    .change_time_us = 0,
    .on_delay_ms = AC_POWER_DELAYED_ON_MS,
    .off_delay_ms = AC_POWER_DELAYED_OFF_MS,
    .gpio = AC_POWER_GPIO,
    .name = "AC LED",
};

static debounce_t s_compressor = {
    .stable_state = false,
    .change_time_us = 0,
    .on_delay_ms = COMPRESSOR_DELAYED_ON_MS,
    .off_delay_ms = COMPRESSOR_DELAYED_OFF_MS,
    .gpio = COMPRESSOR_GPIO,
    .name = "compressor",
};

/* 光敏電阻輸出：冷氣面板 LED 亮 → LDR 低阻 → GPIO 讀到 LOW = ON
 *                  LED 暗 → LDR 高阻 → GPIO 讀到 HIGH = OFF
 * （LDR 分壓 + 內部 pull-up，導通時拉低，對應 ESPHome input + pullup + inverted） */
static inline bool read_gpio_state(gpio_num_t g)
{
    return gpio_get_level(g) == 0;  /* active-low: LOW=ON, HIGH=OFF */
}

static void debounce_update(debounce_t *d, gpio_state_cb_t cb)
{
    bool now = read_gpio_state(d->gpio);
    int64_t now_us = esp_timer_get_time();

    if (now != d->raw_level) {
        d->raw_level = now;
        d->change_time_us = now_us;
    }

    /* 計算是否已過了去抖動時間 */
    uint32_t delay = d->raw_level ? d->on_delay_ms : d->off_delay_ms;
    int64_t elapsed_ms = (now_us - d->change_time_us) / 1000;

    if (d->raw_level != d->stable_state && elapsed_ms >= delay) {
        d->stable_state = d->raw_level;
        ESP_LOGI(TAG, "%s state -> %s", d->name, d->stable_state ? "ON" : "OFF");
        if (cb) cb(d->stable_state);
    }
}

static void gpio_monitor_task(void *arg)
{
    while (true) {
        debounce_update(&s_ac_power, s_ac_power_cb);
        debounce_update(&s_compressor, s_compressor_cb);
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

esp_err_t gpio_monitor_init(void)
{
    /* 設定 GPIO 為輸入 + pull-up（移植自 ESPHome mode: input + pullup） */
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << AC_POWER_GPIO) | (1ULL << COMPRESSOR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 初始化 raw level 和 stable_state（開機時直接同步 GPIO 實際狀態） */
    s_ac_power.raw_level = read_gpio_state(s_ac_power.gpio);
    s_ac_power.stable_state = s_ac_power.raw_level;
    s_ac_power.change_time_us = esp_timer_get_time();
    s_compressor.raw_level = read_gpio_state(s_compressor.gpio);
    s_compressor.stable_state = s_compressor.raw_level;
    s_compressor.change_time_us = esp_timer_get_time();

    BaseType_t ok = xTaskCreate(gpio_monitor_task, "gpio_mon", 3072, NULL, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate gpio_monitor_task failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "GPIO monitor ready: AC panel LED=GPIO%d, compressor=GPIO%d",
             AC_POWER_GPIO, COMPRESSOR_GPIO);
    return ESP_OK;
}

void gpio_monitor_register_ac_power_cb(void (*cb)(bool state))
{
    s_ac_power_cb = cb;
}

void gpio_monitor_register_compressor_cb(void (*cb)(bool state))
{
    s_compressor_cb = cb;
}

void gpio_monitor_trigger_current_state(void)
{
    /* 將目前穩定狀態餵給已註冊的 callback（解決 callback 註冊前初始狀態已偵測的問題） */
    if (s_ac_power_cb) {
        s_ac_power_cb(s_ac_power.stable_state);
    }
    if (s_compressor_cb) {
        s_compressor_cb(s_compressor.stable_state);
    }
}

/* ========== gpio_read console 指令：讀取 GPIO10/GPIO11 raw level + stable_state ========== */
#include <esp_matter_console.h>
#include <esp_timer.h>

static esp_err_t gpio_read_handler(int argc, char **argv)
{
    bool ac_raw = (gpio_get_level(AC_POWER_GPIO) == 0);  /* active-low: LOW=ON */
    bool comp_raw = (gpio_get_level(COMPRESSOR_GPIO) == 0);
    ESP_LOGI(TAG, "GPIO read: AC(LED=GPIO%d) raw_level=%d stable_state=%d (gpio_pin=%d), "
             "COMP(GPIO%d) raw_level=%d stable_state=%d (gpio_pin=%d)",
             AC_POWER_GPIO, ac_raw, s_ac_power.stable_state, gpio_get_level(AC_POWER_GPIO),
             COMPRESSOR_GPIO, comp_raw, s_compressor.stable_state, gpio_get_level(COMPRESSOR_GPIO));
    return ESP_OK;
}

namespace esp_matter {
namespace console {
static const command_t gpio_mon_commands[] = {
    {
        .name = "gpio_read",
        .description = "讀取 GPIO10 (AC power LDR) / GPIO11 (compressor) raw 與 stable 狀態",
        .handler = gpio_read_handler,
    },
};
} // namespace console
} // namespace esp_matter

esp_err_t gpio_monitor_console_init(void)
{
    return esp_matter::console::add_commands(esp_matter::console::gpio_mon_commands,
                                             sizeof(esp_matter::console::gpio_mon_commands) / sizeof(esp_matter::console::command_t));
}
