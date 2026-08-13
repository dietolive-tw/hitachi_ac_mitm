/*
 * Hitachi AC MITM - app_priv.h
 *
 * 改造自 esp-matter/examples/room_air_conditioner/main/app_priv.h
 *
 * 硬體架構（方案 B：非侵入式並聯，移植自 ESPHome yaml）：
 *   GPIO4  ← 獨立 IR Receiver 模組 (VS1838B，並聯監聽遙控器)
 *            ─ VS1838B 模組內建 10kΩ 上拉，OUT 主動低電位
 *            ─ 3.3V 供電直接接；5V 供電需 2.2kΩ/3.3kΩ 分壓到 ~3V
 *   GPIO5  → IR LED (指向原裝 IR Receiver，HA/Matter 控制時才發射)
 *            ─ 必須串限流電阻！R = (3.3V − V_F) / I_F，V_F ≈ 1.3V
 *            ─ 直接驅動：150Ω~200Ω（I_F ≈ 10~15mA，室內 1~3m）
 *            ─ 大功率/長距離：加 NPN 電晶體（基極 1kΩ，集極 10Ω 限流）
 *            ─ 38kHz/50% duty 載波，平均電流 = 峰值 × 50%
 *   GPIO10 ← 冷氣面板指示燈 LED 光敏電阻 (LDR 分壓，active-low + 內部 pull-up) = 冷氣電源狀態
 *   GPIO11 ← 220V 壓縮機光耦 = 壓縮機運轉狀態
 *   GPIO1  ← DS18B20 1-Wire 室溫感測器 (需 4.7kΩ 外部上拉到 3.3V)
 *   GPIO8  → WS2812B 狀態指示燈 (沿用 ESP32-C6 DevKitC-1 板載 LED)
 *   GPIO9  ← Boot 按鍵 (沿用 DevKitC-1 預設，作為 Matter commissioning/reset)
 */

#pragma once

#include <esp_err.h>
#include <esp_matter.h>
#include "driver/gpio.h"        /* GPIO_NUM_* 巨集 */

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

/* ========== GPIO 定義（移植自 ESPHome yaml） ========== */
#define IR_RECEIVER_GPIO       GPIO_NUM_4   /* VS1838B 並聯監聽 (模組內建 10kΩ 上拉) */
#define IR_TRANSMITTER_GPIO    GPIO_NUM_5   /* IR LED 發射 (須串 150Ω~200Ω 限流電阻) */
#define AC_POWER_GPIO          GPIO_NUM_10  /* 冷氣面板 LED 光敏電阻 = 冷氣電源狀態 */
#define COMPRESSOR_GPIO        GPIO_NUM_11  /* 220V 壓縮機光耦 */
#define DS18B20_GPIO           GPIO_NUM_1   /* 1-Wire 室溫感測器 (需 4.7kΩ 上拉) */
#define BUTTON_GPIO            GPIO_NUM_9   /* DevKitC-1 Boot 鍵 */
#define WS2812B_LED_GPIO       GPIO_NUM_8   /* WS2812B 狀態指示燈 */

/* ========== IR 協定參數 ========== */
#define IR_CARRIER_FREQ_HZ     38000        /* 38kHz 載波 */
#define IR_CARRIER_DUTY_PCT     50           /* 50% duty cycle（官方 IRremoteESP8266 kDutyDefault=50）
                                              * 多數 IR 接收器（含冷氣）設計 for 30-50% duty；
                                              * 75% duty 會使 AC 接收器 bandpass/AGC 失常。
                                              * 100Ω 電阻：I_peak ≈ (3.3-1.4)/100 ≈ 19mA
                                              * 50% duty：I_avg_mark ≈ 9.5mA（足夠 1-3m 室內距離） */
/* HITACHI_AC1 = 104 bits → 211 個 us timing ≈ 106 個 RMT symbol。
 * 設 1024 同時容納 symbol 與 timing。 */
#define IR_RAW_MAX_TIMINGS     1024          /* 單次接收最大 timing/symbol 數 */

/* ========== DS18B20 參數 ========== */
#define DS18B20_READ_INTERVAL_MS  2000       /* 每 2 秒讀一次溫度 */

/* ========== 預設屬性值 ========== */
#define DEFAULT_TARGET_TEMP_C  26            /* 預設目標溫度 26°C */
#define DEFAULT_LOCAL_TEMP_C   26            /* 預設室溫 26°C（DS18B20 讀到後會更新） */

/* ========== Endpoint ID（在 app_main 建立後填入） ========== */
extern uint16_t room_air_conditioner_endpoint_id;
extern uint16_t ac_power_contact_endpoint_id;
extern uint16_t compressor_contact_endpoint_id;

typedef void *app_driver_handle_t;

/* ========== Driver 介面 ========== */

/**
 * 初始化所有硬體 driver（IR 收發、GPIO 監控、DS18B20、按鍵）
 * 回傳一個 opaque handle，供 app_attribute_update_cb 使用。
 */
app_driver_handle_t app_driver_hitachi_ac_init(void);

/**
 * 初始化 Boot 按鍵（沿用 esp-matter app_reset 流程）
 */
app_driver_handle_t app_driver_button_init(void);

/**
 * Matter attribute 更新回呼：當 controller 寫入 Thermostat / FanControl 時觸發
 * 負責驅動 IR LED 發射對應編碼（Phase 2 解碼後啟用）。
 */
esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val);

/**
 * 將 driver 設為預設值（開機時呼叫）
 */
esp_err_t app_driver_hitachi_ac_set_defaults(uint16_t endpoint_id);

/* ========== 硬體狀態 → Matter attribute 同步 ========== */

/**
 * 由 GPIO ISR / polling task 呼叫，把風扇繼電器狀態推到 contact_sensor endpoint
 */
void app_driver_update_ac_power_state(bool power_on);

/**
 * 由 GPIO ISR / polling task 呼叫，把壓縮機狀態推到 contact_sensor endpoint
 */
void app_driver_update_compressor_state(bool running);

/**
 * 由 DS18B20 讀取 task 呼叫，把室溫推到 temperature_sensor 與 thermostat.local_temperature
 * @param temp_c 攝氏溫度（float）
 */
void app_driver_update_temperature(float temp_c);

/* ========== WS2812B 狀態指示燈 ========== */

/**
 * 狀態指示燈狀態（對應不同系統狀態顯示不同顏色/閃爍）
 */
typedef enum {
    STATUS_LED_OFF,              /* 熄燈：未啟動或故障 */
    STATUS_LED_BOOT,             /* 紫色：燒錄完成/啟動中 */
    STATUS_LED_COMMISSIONING,    /* 藍色閃爍：配對中（commissioning window 開啟） */
    STATUS_LED_CONNECTED,        /* 綠色：已加入 fabric（配對完成） */
    STATUS_LED_AC_ON,            /* 紅色：冷氣運轉中（風扇繼電器 ON） */
    STATUS_LED_IDENTIFY,         /* 黃色閃爍：識別中（Matter identify effect） */
} status_led_state_t;

/**
 * 初始化 WS2812B 狀態指示燈（使用 RMT TX channel 1）
 * @return ESP_OK 成功
 */
esp_err_t status_led_init(void);

/**
 * 設定狀態指示燈狀態（內部會處理閃爍動畫）
 * @param state 目標狀態
 */
void status_led_set_state(status_led_state_t state);

/**
 * 識別回呼：Matter identify cluster 觸發時呼叫
 * @param start true=開始識別閃爍, false=停止
 */
void status_led_identify(bool start);

/* ========== IR 收發（供 app_driver 內部與 Phase 2 使用） ========== */

/**
 * 發射一段 raw IR timing（單位 us），38kHz 載波
 * @param timings 正負交替的時長陣列（mark/space/mark/space...）
 * @param count  timings 數量
 */
esp_err_t ir_transmit_raw(const uint32_t *timings, size_t count);

/**
 * 取得最近一次接收到的 raw IR timing（供 Phase 2 解碼用）
 * @param out_buf 輸出緩衝區
 * @param buf_size 緩衝區可容納的 timing 數
 * @param out_count 實際寫入的數量
 * @return ESP_OK 有新資料；ESP_ERR_NOT_FOUND 無資料
 */
esp_err_t ir_get_last_received(uint32_t *out_buf, size_t buf_size, size_t *out_count);

/**
 * 取得 RMT RX callback 診斷計數（供排查 partial RX 截斷問題）
 * @param cb_count callback 總觸發次數
 * @param cb_partial is_last=0（buffer 滿，partial）次數
 * @param cb_last is_last=1（idle 結束）次數
 * @param cb_max_accum 累積 symbols 最高水位
 * @param last_cb_n 最近一次 callback 的 symbol 數
 */
void ir_rx_get_diag(uint32_t *cb_count, uint32_t *cb_partial, uint32_t *cb_last,
                    uint32_t *cb_max_accum, size_t *last_cb_n, uint32_t *cb_rearm);

/**
 * 超時 flush：若距最後一次 RMT callback 超過 150ms 且有累積資料，
 * 展平並存到 last_timings。用於多段訊框的最後一段（無後續 callback 觸發 complete）。
 * @return true 表示有 flush 出新訊框
 */
bool ir_rx_flush_stale(void);

/**
 * 清除所有 RX 狀態：累積緩衝、last_timings、callback 時間戳、診斷計數。
 * 用於 loopback 測試前確保 RX 狀態乾淨。
 */
void ir_rx_clear(void);

/**
 * 停止 RMT RX 硬體（rmt_disable）：TX 前呼叫，避免接收自己的訊號。
 * @return ESP_OK 成功
 */
esp_err_t ir_rx_stop(void);

/**
 * 重新啟動 RMT RX 硬體（rmt_enable + 清除 + re-arm）：TX confirm 後呼叫。
 * @return ESP_OK 成功
 */
esp_err_t ir_rx_start(void);

/**
 * IR console commands 初始化（供 app_main 註冊）
 */
esp_err_t ir_console_init(void);

/**
 * 註冊 app_driver console 指令（contact_set 等）
 */
esp_err_t app_driver_console_init(void);

/**
 * 啟動 USB Serial/JTAG console input task
 * 讀取 /dev/ttyACM0 輸入並轉發給 esp_console，繞過 linenoise 的 UART0 stdin 限制
 */
esp_err_t usb_jtag_console_start(void);

/* ========== GPIO 監控（供 app_driver 註冊 callback） ========== */

/**
 * 註冊風扇繼電器狀態變化 callback
 */
void gpio_monitor_register_ac_power_cb(void (*cb)(bool state));

/**
 * 註冊壓縮機狀態變化 callback
 */
void gpio_monitor_register_compressor_cb(void (*cb)(bool state));

/**
 * 將目前 GPIO 穩定狀態餵給已註冊的 callback
 * 用於 callback 註冊後補觸發初始狀態（解決註冊前初始狀態已偵測的時序問題）
 */
void gpio_monitor_trigger_current_state(void);

/**
 * 註冊 gpio_monitor console 指令（gpio_read 等）
 */
esp_err_t gpio_monitor_console_init(void);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
    {                                                                                   \
        .radio_mode = RADIO_MODE_NATIVE,                                                \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                            \
    {                                                                                   \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                              \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif
