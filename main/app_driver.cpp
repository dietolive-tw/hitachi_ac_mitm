/*
 * Hitachi AC MITM - app_driver.cpp
 *
 * 改造自 esp-matter/examples/room_air_conditioner/main/app_driver.cpp
 *
 * 職責：
 *   1. app_driver_hitachi_ac_init()  → 初始化 IR / DS18B20 / GPIO 監控 driver
 *   2. app_driver_button_init()      → Boot 按鍵（GPIO9）commissioning/reset
 *   3. app_driver_attribute_update() → 處理 controller 寫入 Thermostat / FanControl
 *                                       Phase 1：只 log，Phase 2：發射 IR
 *   4. app_driver_hitachi_ac_set_defaults() → 開機預設值
 *   5. app_driver_update_*()        → 硬體狀態 → Matter attribute 同步
 *                                       (DS18B20 溫度、風扇繼電器、壓縮機)
 */

#include <stdlib.h>
#include <string.h>

#include <esp_log.h>
#include <esp_matter.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <app_priv.h>
#include <device.h>
#include "ir_hitachi_ac1.h"

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <app/clusters/boolean-state-server/BooleanStateCluster.h>
#include <app/clusters/fan-control-server/CodegenIntegration.h>
#include <app/clusters/temperature-measurement-server/CodegenIntegration.h>
#include <app/server-cluster/ServerClusterInterfaceRegistry.h>
#include <esp_matter_console.h>
#include <esp_matter_data_model_provider.h>

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::attribute;

static const char *TAG = "app_driver";

/* Endpoint IDs（app_main.cpp 定義，這裡 extern 引用） */
extern uint16_t room_air_conditioner_endpoint_id;
extern uint16_t ac_power_contact_endpoint_id;
extern uint16_t compressor_contact_endpoint_id;

/* ========== 各 driver 的 init / callback 註冊（在各自 .cpp 實作，C++ linkage） ========== */
extern esp_err_t ir_driver_init(void);
extern esp_err_t ir_transmit_raw(const uint32_t *timings, size_t count);
extern esp_err_t ir_get_last_received(uint32_t *out_buf, size_t buf_size, size_t *out_count);
extern void ir_rx_get_diag(uint32_t *cb_count, uint32_t *cb_partial, uint32_t *cb_last,
                           uint32_t *cb_max_accum, size_t *last_cb_n, uint32_t *cb_rearm);
extern bool ir_rx_flush_stale(void);
extern bool ir_rx_is_exclusive(void);
extern void ir_rx_set_exclusive(bool exclusive);
extern void ir_rx_clear(void);
extern esp_err_t ir_rx_stop(void);
extern esp_err_t ir_rx_start(void);
extern esp_err_t ds18b20_driver_init(void);
extern void ds18b20_register_update_cb(void (*cb)(float temp_c));
extern esp_err_t gpio_monitor_init(void);
extern void gpio_monitor_register_ac_power_cb(void (*cb)(bool state));
extern void gpio_monitor_register_compressor_cb(void (*cb)(bool state));

/* ========== Phase 2：冷氣真實狀態（MITM 核心資料） ==========
 * 這個 state 是冷氣的「真實狀態」：
 *   - 由監聽遙控器解碼更新（app_driver_ir_rx_task）
 *   - 由 Matter controller 命令更新（app_driver_attribute_update）
 * 每次變動後編碼成 IR timing 並透過 ir_transmit_raw() 發射給冷氣。
 */
static hitachi_ac1_state_t s_ac_state;
static bool s_ac_state_inited = false;
/* AC 電源狀態從 GPIO 偵測取得 */
static bool s_ac_gpio_power = false;
/* 開/關機命令已發射但尚未被 GPIO 確認 — 防止 coalesce 視窗外的後續命令重複加 PowerToggle */
static bool s_power_toggle_pending = false;

/* IR 編碼緩衝（HITACHI_AC1 = 211 個 timing） */
static uint32_t s_ir_tx_buf[1024];

/* 從 RX 接收的 timing 緩衝 */
static uint32_t s_ir_rx_buf[IR_RAW_MAX_TIMINGS];

/* 防止監聽解碼更新 attribute 時觸發 attribute_update 回呼再發 IR（迴圈） */
static bool s_suppress_ir_tx = false;

/* 上次已發射的 state（用於 skip 重複 TX） */
static hitachi_ac1_state_t s_last_sent_state;
static bool s_last_sent_valid = false;

/* ========== Boot 按鍵：Toggle SystemMode (Off ↔ Cool) ========== */
static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed (冷氣 SystemMode)");
    uint16_t endpoint_id = room_air_conditioner_endpoint_id;
    uint32_t cluster_id = Thermostat::Id;
    uint32_t attribute_id = Thermostat::Attributes::SystemMode::Id;

    /* 在 Matter thread 上更新 attribute（button cb 來自 button task） */
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, cluster_id, attribute_id]() {
        attribute_t *attribute = attribute::get(endpoint_id, cluster_id, attribute_id);
        if (attribute == nullptr) {
            ESP_LOGE(TAG, "SystemMode attribute not found for endpoint %d", endpoint_id);
            return;
        }
        esp_matter_attr_val_t val;
        attribute::get_val(attribute, &val);
        /* Toggle: Off(0) ↔ Cool(3) */
        val.val.u8 = (val.val.u8 == 0) ? 3 : 0;
        attribute::update(endpoint_id, cluster_id, attribute_id, &val);
    });
}

/* ========== Phase 2：attribute update → 修改 state → 編碼 → 發射 IR ========== */

/* 前向宣告（定義在後方） */
static esp_err_t app_driver_ir_send_state(void);
static void app_driver_sync_attribute_from_state(void);
static uint8_t hitachi_fan_to_matter(uint8_t h_fan);
static uint8_t fan_mode_to_percent(uint8_t m_fan);

/* TX generation counter：只讓最新一次 TX 的 confirm task 執行 resume RX，
 * 避免連續下達命令時舊的 confirm task 提前 resume RX。 */
static volatile uint32_t s_tx_generation = 0;

/* ========== IR TX 合併視窗（coalesce） ==========
 * HITACHI_AC1 一包 13-byte 訊框即承載所有欄位（power/temp/mode/fan...），
 * 不需要為每個 attribute write 各發一包。Matter controller 在單一使用者操作中
 * 可能連續發送多個獨立命令（如 WriteRequest SystemMode + InvokeCommand Toggle，
 * 兩者間隔典型 ~100-150ms）。這裡用 one-shot esp_timer 把視窗內（預設 200ms）
 * 的所有 attribute write 合併成單次 IR TX：每個 setter 重置計時器，最後一次
 * 寫入後 200ms 才發射。set_defaults 仍直接呼叫 app_driver_ir_send_state() 立即發射。 */
static esp_timer_handle_t s_tx_coalesce_timer = NULL;
#define TX_COALESCE_MS  200  /* 合併視窗：200ms 內的多個 attribute write 只發一次 IR */

static void app_driver_tx_coalesce_cb(void *arg)
{
    ESP_LOGI(TAG, "TX coalesce window expired, sending IR");
    app_driver_ir_send_state();
}

/* 排程合併 TX：取消任何待發的計時器並重新啟動一次性計時器。 */
static esp_err_t app_driver_schedule_ir_send(void)
{
    if (!s_ac_state_inited) return ESP_ERR_INVALID_STATE;
    if (s_suppress_ir_tx) return ESP_OK;
    if (s_tx_coalesce_timer == NULL) return ESP_ERR_INVALID_STATE;
    esp_timer_stop(s_tx_coalesce_timer);
    return esp_timer_start_once(s_tx_coalesce_timer, TX_COALESCE_MS * 1000);
}

/* IR TX 完成後延遲確認 task：等 2 秒讓 AC 反應，然後檢查 AC GPIO 並同步
 * final state 到 Matter attribute，最後恢復 IR RX 硬體。
 * 若期間有新的 TX（generation 不符），則跳過 resume RX。 */
static void app_driver_ir_tx_confirm_task(void *arg)
{
    uint32_t my_gen = (uint32_t)(uintptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));  /* 等 2 秒讓 AC 反應 */

    if (my_gen != s_tx_generation) {
        ESP_LOGI(TAG, "TX confirm (gen=%u) superseded by gen=%u, skip resume",
                 (unsigned)my_gen, (unsigned)s_tx_generation);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TX confirm: check AC GPIO and sync final state (power=%s)",
             s_ac_gpio_power ? "ON" : "OFF");

    /* 遙控器模式：Pwr bit 始終 0，實際電源狀態由 GPIO 判讀。
     * sync_attribute_from_state 用 s_ac_gpio_power 決定 SystemMode：
     * - AC 收到命令 (GPIO=ON) → SystemMode 同步 s_ac_state.mode
     * - AC 沒收到命令 (GPIO=OFF) → SystemMode=Off（revert 使用者的樂觀設定）
     * invalidate last_sent 讓下次命令可重試。 */
    if (s_ac_state_inited && !s_ac_gpio_power) {
        s_last_sent_valid = false;
    }

    app_driver_sync_attribute_from_state();

    /* 清除 power_toggle_pending：confirm 已檢查過 GPIO，AC 是否收到命令已確認。
     * 若 GPIO 仍 OFF (AC 沒收到)，清掉此 flag 讓下次命令可重新加 PowerToggle 重試；
     * 在 confirm 之前保留 true 可避免 0~2 秒內連續命令 double-toggle AC。 */
    s_power_toggle_pending = false;

    /* 恢復 IR RX 硬體：重新 enable + 清除 + re-arm */
    ir_rx_start();
    ir_rx_set_exclusive(false);
    ESP_LOGI(TAG, "IR RX resumed");
    vTaskDelete(NULL);
}

/* 把當前 s_ac_state 編碼並透過 IR LED 發射給冷氣
 * 流程：停止 IR RX 硬體 → 清除 RX 緩衝 → 編碼 + 發射 → 啟動 confirm task
 * confirm task 等 2 秒後檢查 AC GPIO、同步 final state、恢復 IR RX */
static esp_err_t app_driver_ir_send_state(void)
{
    if (!s_ac_state_inited) return ESP_ERR_INVALID_STATE;
    if (s_suppress_ir_tx) return ESP_OK;  /* 監聽更新中，不回發 */

    /* Skip 重複 TX：若 s_ac_state 與上次已發射的 state 完全相同（含 PowerToggle
     * bit），表示自上次 TX 後無新變化，不需要再發射。 */
    if (s_last_sent_valid &&
        memcmp(&s_ac_state, &s_last_sent_state, sizeof(hitachi_ac1_state_t)) == 0) {
        ESP_LOGI(TAG, "IR TX skipped: state unchanged since last send");
        return ESP_OK;
    }

    /* 停止 IR RX 硬體：避免 RMT 接收自己的 TX 訊號 */
    ir_rx_set_exclusive(true);
    ir_rx_stop();
    ir_rx_clear();

    /* 編碼 + 發射 */
    size_t n = 0;
    esp_err_t err = hitachi_ac1_encode(&s_ac_state, s_ir_tx_buf, sizeof(s_ir_tx_buf) / sizeof(uint32_t), &n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hitachi_ac1_encode failed: %s", esp_err_to_name(err));
        ir_rx_start();
        ir_rx_set_exclusive(false);
        return err;
    }
    ESP_LOGI(TAG, "IR TX: %u timings", (unsigned)n);
    hitachi_ac1_state_log(&s_ac_state);
    err = ir_transmit_raw(s_ir_tx_buf, n);
    if (err != ESP_OK) {
        ir_rx_start();
        ir_rx_set_exclusive(false);
        return err;
    }

    /* 發射後清除 PowerToggle */
    hitachi_ac1_set_power_toggle(&s_ac_state, false);

    /* 記錄已發射的 state（PowerToggle 清除後），供下次 skip 比對 */
    memcpy(&s_last_sent_state, &s_ac_state, sizeof(hitachi_ac1_state_t));
    s_last_sent_valid = true;

    /* 啟動延遲確認 task：2 秒後檢查 AC GPIO 並同步 final state，然後恢復 RX。 */
    s_tx_generation++;
    if (xTaskCreate(app_driver_ir_tx_confirm_task, "ir_conf", 3072,
                    (void *)(uintptr_t)s_tx_generation, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ir_tx_confirm task, resuming RX immediately");
        ir_rx_start();
        ir_rx_set_exclusive(false);
    }
    return ESP_OK;
}

/* ========== Phase 2：attribute update → 修改 state → 編碼 → 發射 IR ==========
 * 內部 state-only 函式只更新 s_ac_state 不發 TX，供 set_defaults 批次更新後一次發射。
 */

static void app_driver_ac_set_target_temp_state(esp_matter_attr_val_t *val)
{
    int16_t temp_centi = val->val.i16;
    uint8_t target_c = (uint8_t)((temp_centi + 50) / 100);  /* 四捨五入 */
    ESP_LOGI(TAG, "[Phase2] 目標溫度 → %d°C (raw=%d)", target_c, temp_centi);
    hitachi_ac1_set_temp(&s_ac_state, target_c);
}

static void app_driver_ac_set_system_mode_state(esp_matter_attr_val_t *val)
{
    const char *mode_str = "Unknown";
    uint8_t hitachi_mode = HITACHI_AC1_MODE_COOL;
    /* Matter SystemMode: 0=Off, 1=Auto, 3=Cool, 4=Heat, 5=Preconditioning */
    switch (val->val.u8) {
    case 0:
        mode_str = "Off";
        /* 遙控器模式：Pwr bit 始終 0，靠 PwrTgl 切換電源。
         * AC ON → 送 PwrTgl=1 切成 OFF；AC OFF → 不需 toggle（已經是目標狀態）。 */
        if (s_ac_gpio_power && !s_power_toggle_pending) {
            hitachi_ac1_set_power_toggle(&s_ac_state, true);
            s_power_toggle_pending = true;
        }
        ESP_LOGI(TAG, "[Phase2] 系統模式 → %s (raw=%u)", mode_str, val->val.u8);
        /* SystemMode=Off → 同步 FanMode=kOff（AC 關機風扇也停）。
         * ApplyFanModeSideEffects(kOff) 已設 PercentCurrent=0，無需另叫 SetPercentCurrent。 */
        {
            s_suppress_ir_tx = true;
            esp_matter_attr_val_t fan_val = esp_matter_attr_val((uint8_t)FanControl::FanModeEnum::kOff, esp_matter_attr_val::uint_sub_type::k_enum);
            attribute::update(room_air_conditioner_endpoint_id, FanControl::Id, FanControl::Attributes::FanMode::Id, &fan_val);
            s_suppress_ir_tx = false;
        }
        return;
    case 1: mode_str = "Auto"; hitachi_mode = HITACHI_AC1_MODE_COOL; break; /* Hitachi 無 Auto，用 Cool */
    case 3: mode_str = "Cool"; hitachi_mode = HITACHI_AC1_MODE_COOL; break;
    case 4: mode_str = "Heat"; hitachi_mode = HITACHI_AC1_MODE_HEAT; break;
    case 5: mode_str = "Preconditioning"; hitachi_mode = HITACHI_AC1_MODE_DRY; break;
    case 7: mode_str = "Fan"; hitachi_mode = HITACHI_AC1_MODE_FAN; break;
    default: break;
    }
    /* 非 Off 模式：若 AC OFF 且尚未送過開機命令，同時開機。
     * 遙控器模式：Pwr bit 始終 0，靠 PwrTgl 切換電源。
     * AC OFF → 送 PwrTgl=1 切成 ON；AC ON → 不需 toggle（只更新 mode）。 */
    bool added_toggle = false;
    if (!s_ac_gpio_power && !s_power_toggle_pending) {
        hitachi_ac1_set_power_toggle(&s_ac_state, true);
        s_power_toggle_pending = true;
        added_toggle = true;
    }
    hitachi_ac1_set_mode(&s_ac_state, hitachi_mode);
    ESP_LOGI(TAG, "[Phase2] 系統模式 → %s (raw=%u, pwr_tgl=%s)", mode_str, val->val.u8,
             added_toggle ? "Y" : "N");
    /* SystemMode 非 Off → 若 FanMode 目前是 kOff（之前 SystemMode=Off 被聯動），
     * 恢復為 s_ac_state 的實際風速，否則 UI 顯示風扇 Off 但 AC 實際有風。
     * ApplyFanModeSideEffects 對非 kOff 模式不更新 PercentCurrent，需手動設。 */
    {
        attribute_t *fan_attr = attribute::get(room_air_conditioner_endpoint_id, FanControl::Id, FanControl::Attributes::FanMode::Id);
        if (fan_attr != nullptr) {
            esp_matter_attr_val_t cur_fan;
            attribute::get_val(fan_attr, &cur_fan);
            if (cur_fan.val.u8 == (uint8_t)FanControl::FanModeEnum::kOff) {
                s_suppress_ir_tx = true;
                uint8_t matter_fan = hitachi_fan_to_matter(hitachi_ac1_get_fan(&s_ac_state));
                esp_matter_attr_val_t fan_val = esp_matter_attr_val(matter_fan, esp_matter_attr_val::uint_sub_type::k_enum);
                attribute::update(room_air_conditioner_endpoint_id, FanControl::Id, FanControl::Attributes::FanMode::Id, &fan_val);
                {
                    lock::ScopedChipStackLock lock(portMAX_DELAY);
                    auto *fan_cluster = chip::app::Clusters::FanControl::FindClusterOnEndpoint(room_air_conditioner_endpoint_id);
                    if (fan_cluster != nullptr) {
                        fan_cluster->SetPercentCurrent(fan_mode_to_percent(matter_fan));
                        ESP_LOGI(TAG, "[sysmode] PercentCurrent=%u", (unsigned)fan_mode_to_percent(matter_fan));
                    }
                }
                s_suppress_ir_tx = false;
            }
        }
    }
}

/* 公開 setter：更新 state + 排程合併 IR TX */
static esp_err_t app_driver_ac_set_target_temp(esp_matter_attr_val_t *val)
{
    app_driver_ac_set_target_temp_state(val);
    return app_driver_schedule_ir_send();
}

static esp_err_t app_driver_ac_set_system_mode(esp_matter_attr_val_t *val)
{
    app_driver_ac_set_system_mode_state(val);
    return app_driver_schedule_ir_send();
}

/* ========== FanControl: Matter FanMode ↔ Hitachi fan speed ========== */

static uint8_t hitachi_fan_to_matter(uint8_t h_fan)
{
    /* Hitachi fan values: AUTO=1, HIGH=2, MED=4, LOW=8 */
    switch (h_fan) {
    case HITACHI_AC1_FAN_AUTO:  return (uint8_t)FanControl::FanModeEnum::kAuto;
    case HITACHI_AC1_FAN_HIGH:  return (uint8_t)FanControl::FanModeEnum::kHigh;
    case HITACHI_AC1_FAN_MED:   return (uint8_t)FanControl::FanModeEnum::kMedium;
    case HITACHI_AC1_FAN_LOW:   return (uint8_t)FanControl::FanModeEnum::kLow;
    default:                    return (uint8_t)FanControl::FanModeEnum::kAuto;
    }
}

static uint8_t matter_fan_to_hitachi(uint8_t m_fan)
{
    /* Matter FanModeEnum: kOff=0, kLow=1, kMedium=2, kHigh=3, kOn=4, kAuto=5, kSmart=6 */
    switch (m_fan) {
    case (uint8_t)FanControl::FanModeEnum::kLow:    return HITACHI_AC1_FAN_LOW;
    case (uint8_t)FanControl::FanModeEnum::kMedium: return HITACHI_AC1_FAN_MED;
    case (uint8_t)FanControl::FanModeEnum::kHigh:   return HITACHI_AC1_FAN_HIGH;
    case (uint8_t)FanControl::FanModeEnum::kOn:
    case (uint8_t)FanControl::FanModeEnum::kAuto:   return HITACHI_AC1_FAN_AUTO;
    case (uint8_t)FanControl::FanModeEnum::kOff:    return HITACHI_AC1_FAN_LOW; /* Off 不可能，fallback Low */
    default:                                        return HITACHI_AC1_FAN_AUTO;
    }
}

/* Map Matter FanMode → PercentCurrent (0-100).
 * Matches ApplyFanModeSideEffects() PercentSetting targets in FanControlCluster.cpp:
 *   kOff=0, kLow=33, kMedium=66, kHigh=100, kAuto=50 (arbitrary mid-range, since
 *   the cluster leaves PercentCurrent unchanged for kAuto — we provide a sane default
 *   so Apple Home displays the fan as running, not 0%). */
static uint8_t fan_mode_to_percent(uint8_t m_fan)
{
    switch (m_fan) {
    case (uint8_t)FanControl::FanModeEnum::kOff:    return 0;
    case (uint8_t)FanControl::FanModeEnum::kLow:    return 33;
    case (uint8_t)FanControl::FanModeEnum::kMedium: return 66;
    case (uint8_t)FanControl::FanModeEnum::kHigh:   return 100;
    case (uint8_t)FanControl::FanModeEnum::kOn:     return 100;
    case (uint8_t)FanControl::FanModeEnum::kAuto:   return 50;
    case (uint8_t)FanControl::FanModeEnum::kSmart:   return 50;
    default:                                        return 0;
    }
}

static void app_driver_ac_set_fan_mode_state(esp_matter_attr_val_t *val)
{
    /* FanMode=kOff → 關機：等同 SystemMode=Off（AC 沒有獨立風扇開關，
     * 關風扇 = 關整台冷氣）。遙控器模式：Pwr bit 始終 0，靠 PwrTgl 切換。
     * AC ON → 送 PwrTgl=1 切成 OFF；AC OFF → 不需 toggle。
     * 注意：此函式在 PRE_UPDATE callback 中執行（SetFanMode 之前）。
     * ApplyFanModeSideEffects(kOff) 會設 PercentCurrent=0；非 kOff 模式不動 PercentCurrent。 */
    if (val->val.u8 == (uint8_t)FanControl::FanModeEnum::kOff) {
        if (s_ac_gpio_power && !s_power_toggle_pending) {
            hitachi_ac1_set_power_toggle(&s_ac_state, true);
            s_power_toggle_pending = true;
        }
        ESP_LOGI(TAG, "[Phase2] 風速 → Off (= 關機)");
        /* 同步 SystemMode=Off，避免 UI 顯示不一致 */
        s_suppress_ir_tx = true;
        esp_matter_attr_val_t mode_val = esp_matter_attr_val((uint8_t)0, esp_matter_attr_val::uint_sub_type::k_enum);
        attribute::update(room_air_conditioner_endpoint_id, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &mode_val);
        s_suppress_ir_tx = false;
        return;
    }
    uint8_t h_fan = matter_fan_to_hitachi(val->val.u8);
    ESP_LOGI(TAG, "[Phase2] 風速 → Matter=%u Hitachi=%u", val->val.u8, h_fan);
    hitachi_ac1_set_fan(&s_ac_state, h_fan);
    /* SetFanMode 將在 PRE_UPDATE callback 返回後由 codegen WriteAttribute 執行；
     * ApplyFanModeSideEffects 對非 kOff 模式不更新 PercentCurrent，需在此先設好。
     * ScopedChipStackLock: PRE_UPDATE callback 已在 attribute::update 的 lock scope 內，
     * 但 lock tracking 的 re-entrancy guard 使此處可安全再取（ALREADY_TAKEN 為 no-op）。 */
    {
        lock::ScopedChipStackLock lock(portMAX_DELAY);
        auto *fan_cluster = chip::app::Clusters::FanControl::FindClusterOnEndpoint(room_air_conditioner_endpoint_id);
        if (fan_cluster != nullptr) {
            fan_cluster->SetPercentCurrent(fan_mode_to_percent(val->val.u8));
            ESP_LOGI(TAG, "[ctrl] PercentCurrent=%u", (unsigned)fan_mode_to_percent(val->val.u8));
        }
    }
}

static esp_err_t app_driver_ac_set_fan_mode(esp_matter_attr_val_t *val)
{
    app_driver_ac_set_fan_mode_state(val);
    return app_driver_schedule_ir_send();
}

/* ========== Phase 2：監聽遙控器 → 解碼 → 同步 Matter attribute ==========
 * MITM 架構下，原裝遙控器按鍵時 ESP32 並聯監聽，解碼出 state 後
 * 反向同步到 Matter attribute，讓 HA/Apple Home 顯示與遙控器一致。
 */
static void app_driver_sync_attribute_from_state(void)
{
    if (!s_ac_state_inited) return;
    s_suppress_ir_tx = true;  /* 同步中不回發 IR */

    uint16_t ep = room_air_conditioner_endpoint_id;

    /* 目標溫度（Matter 用 0.01°C，i16） */
    int16_t temp_centi = (int16_t)hitachi_ac1_get_temp(&s_ac_state) * 100;
    esp_matter_attr_val_t temp_val = esp_matter_int16(temp_centi);
    attribute::update(ep, Thermostat::Id, Thermostat::Attributes::OccupiedCoolingSetpoint::Id, &temp_val);

    /* 系統模式 - Matter Thermostat 只支援 0=Off, 1=Auto, 3=Cool, 4=Heat, 5=Preconditioning
     * 注意：必須先檢查 s_ac_gpio_power — AC 實際 OFF 時 SystemMode 應為 0 (Off)，
     * 不論 s_ac_state.mode 為何（mode 只代表「上次設定的模式」，不代表 AC 正在運轉）。
     * 遙控器模式：Pwr bit 始終 0，不能用 hitachi_ac1_get_power()，必須用 GPIO 判讀。 */
    uint8_t matter_mode;
    if (!s_ac_gpio_power) {
        matter_mode = 0;  /* Off */
    } else {
        matter_mode = 3;  /* Cool 預設 */
        switch (hitachi_ac1_get_mode(&s_ac_state)) {
        case HITACHI_AC1_MODE_COOL: matter_mode = 3; break;
        case HITACHI_AC1_MODE_HEAT: matter_mode = 4; break;
        case HITACHI_AC1_MODE_FAN:  /* FAN/DRY - Matter 無對應，用 Auto fallback */
        case HITACHI_AC1_MODE_AUTO: matter_mode = 1; break;
        default: break;
        }
    }
    esp_matter_attr_val_t mode_val = esp_matter_attr_val(matter_mode, esp_matter_attr_val::uint_sub_type::k_enum);
    esp_err_t mode_ret = attribute::update(ep, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &mode_val);
    ESP_LOGI(TAG, "[sync] SystemMode=%u ret=%d", matter_mode, (int)mode_ret);

    /* FanMode - AC OFF → kOff，AC ON → 從 Hitachi fan 映射 */
    uint8_t matter_fan;
    uint8_t h_fan;
    if (!s_ac_gpio_power) {
        matter_fan = (uint8_t)FanControl::FanModeEnum::kOff;
        h_fan = 0xFF;
    } else {
        h_fan = hitachi_ac1_get_fan(&s_ac_state);
        matter_fan = hitachi_fan_to_matter(h_fan);
    }
    esp_matter_attr_val_t fan_val = esp_matter_attr_val(matter_fan, esp_matter_attr_val::uint_sub_type::k_enum);
    esp_err_t fan_ret = attribute::update(ep, FanControl::Id, FanControl::Attributes::FanMode::Id, &fan_val);
    ESP_LOGI(TAG, "[sync] FanMode: h_fan=0x%X matter=%u ret=%d (gpio=%d)",
             (unsigned)h_fan, matter_fan, (int)fan_ret, (int)s_ac_gpio_power);

    /* PercentCurrent — FanControlCluster::SetFanMode (via codegen WriteAttribute) updates
     * mFanMode and mPercentSetting, but does NOT update mPercentCurrent for non-Off modes
     * (per spec, the application sets PercentCurrent to reflect actual fan speed).
     * Apple Home's fan UI uses PercentCurrent for the speed display, so we must set it
     * here using the codegen API (attribute::update would desync — PercentCurrent is
     * non-writable, so set_val_internal writes esp-matter storage but ReadAttribute
     * reads the codegen cluster member mPercentCurrent).
     * ScopedChipStackLock: FindClusterOnEndpoint + SetPercentCurrent touch the codegen
     * cluster object directly (not via attribute::update which acquires the lock itself). */
    uint8_t percent = fan_mode_to_percent(matter_fan);
    {
        lock::ScopedChipStackLock lock(portMAX_DELAY);
        auto *fan_cluster = chip::app::Clusters::FanControl::FindClusterOnEndpoint(ep);
        if (fan_cluster != nullptr) {
            fan_cluster->SetPercentCurrent(percent);
            ESP_LOGI(TAG, "[sync] PercentCurrent=%u", (unsigned)percent);
        } else {
            ESP_LOGW(TAG, "[sync] FanControl cluster not found for endpoint %d", ep);
        }
    }

    s_suppress_ir_tx = false;
}

/* IR 接收 task：輪詢 ir_get_last_received()，解碼 HITACHI_AC1 後同步 attribute */
static void app_driver_ir_rx_task(void *arg)
{
    uint32_t last_diag_count = 0;
    while (true) {
        /* 每秒 log 一次 callback 診斷 */
        uint32_t cb_count, cb_partial, cb_last, cb_max_accum, cb_rearm;
        size_t last_cb_n;
        ir_rx_get_diag(&cb_count, &cb_partial, &cb_last, &cb_max_accum, &last_cb_n, &cb_rearm);
        if (cb_count != last_diag_count) {
            last_diag_count = cb_count;
            ESP_LOGI(TAG, "RMT diag: cb=%u (partial=%u last=%u rearm=%u) max_accum=%u last_n=%u",
                     (unsigned)cb_count, (unsigned)cb_partial, (unsigned)cb_last,
                     (unsigned)cb_rearm, (unsigned)cb_max_accum, (unsigned)last_cb_n);
        }

        /* 超時 flush：多段訊框的最後一段不會觸發 frame_complete，靠超時 flush */
        ir_rx_flush_stale();

        /* console 指令（如 ir_loopback）使用 exclusive 模式時，跳過讀取 */
        if (ir_rx_is_exclusive()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        size_t count = 0;
        esp_err_t err = ir_get_last_received(s_ir_rx_buf, IR_RAW_MAX_TIMINGS, &count);
        if (err == ESP_OK && count >= 211 && count <= 212) {
            hitachi_ac1_state_t decoded;
            err = hitachi_ac1_decode(s_ir_rx_buf, count, &decoded);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "IR RX: %u timings (gpio_power=%d)", (unsigned)count, (int)s_ac_gpio_power);
                hitachi_ac1_state_log(&decoded);
                memcpy(&s_ac_state, &decoded, sizeof(s_ac_state));
                app_driver_sync_attribute_from_state();
            } else {
                ESP_LOGW(TAG, "IR decode failed: %s", esp_err_to_name(err));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    esp_err_t err = ESP_OK;

    if (s_suppress_ir_tx) return ESP_OK;

    if (endpoint_id == room_air_conditioner_endpoint_id) {
        if (cluster_id == Thermostat::Id) {
            /* Thermostat cluster 沒有 TargetTemperature；冷氣用 OccupiedCoolingSetpoint 作目標溫度 */
            if (attribute_id == Thermostat::Attributes::OccupiedCoolingSetpoint::Id) {
                err = app_driver_ac_set_target_temp(val);
            } else if (attribute_id == Thermostat::Attributes::SystemMode::Id) {
                err = app_driver_ac_set_system_mode(val);
            }
        } else if (cluster_id == FanControl::Id) {
            if (attribute_id == FanControl::Attributes::FanMode::Id) {
                err = app_driver_ac_set_fan_mode(val);
            }
        }
    }

    return err;
}

esp_err_t app_driver_hitachi_ac_set_defaults(uint16_t endpoint_id)
{
    esp_err_t err = ESP_OK;
    esp_matter_attr_val_t val;

    /* 批次更新所有 state（不個別發 TX），最後也不發射 — 開機時只同步內部 state，
     * 不對冷氣發 IR（避免開機閃爍 AC 或意外開機）。等 controller 或遙控器觸發才發射。 */
    attribute_t *attribute = attribute::get(endpoint_id, Thermostat::Id, Thermostat::Attributes::OccupiedCoolingSetpoint::Id);
    if (attribute != nullptr) {
        attribute::get_val(attribute, &val);
        app_driver_ac_set_target_temp_state(&val);
    }

    attribute = attribute::get(endpoint_id, Thermostat::Id, Thermostat::Attributes::SystemMode::Id);
    if (attribute != nullptr) {
        attribute::get_val(attribute, &val);
        app_driver_ac_set_system_mode_state(&val);
    }

    return err;
}

app_driver_handle_t app_driver_hitachi_ac_init(void)
{
    ESP_LOGI(TAG, "Initializing Hitachi AC MITM drivers...");

    /* IR 收發 driver */
    if (ir_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "IR driver init failed");
    }

    /* DS18B20 1-Wire 溫度感測器 driver */
    if (ds18b20_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "DS18B20 driver init failed");
    }

    /* GPIO 監控 driver（光敏電阻 + 壓縮機光耦） */
    if (gpio_monitor_init() != ESP_OK) {
        ESP_LOGE(TAG, "GPIO monitor init failed");
    }

    /* Phase 2：初始化冷氣真實狀態
     * Model B（遙控器實際設定），預設 Auto 25°C Auto fan Power ON
     * state_reset() 預設 Model A，必須覆寫為 B 與冷氣匹配 */
    hitachi_ac1_state_reset(&s_ac_state);
    hitachi_ac1_set_model(&s_ac_state, HITACHI_AC1_MODEL_B);
    s_ac_state_inited = true;
    s_ac_gpio_power = false;  /* 初始假設 AC OFF，等 GPIO 回調更新 */
    ESP_LOGI(TAG, "HITACHI_AC1 state inited (Model B, Auto 25C Auto fan Power ON)");

    /* 建立 coalesce timer（one-shot，200ms） */
    esp_timer_create_args_t timer_args = {
        .callback = app_driver_tx_coalesce_cb,
        .name = "tx_coalesce"
    };
    esp_timer_create(&timer_args, &s_tx_coalesce_timer);

    /* 啟動 IR 接收 task：監聽遙控器 → 解碼 → 同步 Matter attribute */
    xTaskCreate(app_driver_ir_rx_task, "ir_rx", 4096, NULL, 5, NULL);

    /* 回傳非 NULL 的 opaque handle（這裡沒有單一硬體 handle，用 (void *)1 表示已初始化） */
    return (app_driver_handle_t)1;
}

app_driver_handle_t app_driver_button_init(void)
{
    /* Initialize button */
    button_handle_t handle = NULL;
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = button_driver_get_config();

    if (iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button device");
        return NULL;
    }

    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);
    return (app_driver_handle_t)handle;
}

/* ========== 硬體狀態 → Matter attribute 同步 ========== */

/**
 * 把風扇繼電器狀態推到 contact_sensor endpoint 的 boolean_state.state_value
 * 由 gpio_monitor polling task 呼叫，需 ScheduleLambda 切到 Matter thread
 *
 * 注意：BooleanState cluster 採用 codegen data model provider，StateValue 由
 * BooleanStateCluster::mStateValue 內部管理，不能用 attribute::update（會回傳
 * ESP_ERR_NOT_SUPPORTED）。正確做法是透過 data model provider registry 取得
 * BooleanStateCluster instance 後呼叫 SetStateValue()。
 */
void app_driver_update_ac_power_state(bool power_on)
{
    s_ac_gpio_power = power_on;  /* 儲存 GPIO 電源狀態（實際電源狀態的唯一來源） */
    s_power_toggle_pending = false;  /* GPIO 已確認電源狀態，清除 pending flag */
    /* 遙控器模式：Pwr bit 始終 0，不同步 s_ac_state.power。
     * GPIO 變化代表 AC 實際切換了，invalidate last_sent 讓下次命令一定發 TX。 */
    if (s_ac_state_inited) {
        s_last_sent_valid = false;
    }
    uint16_t contact_ep = ac_power_contact_endpoint_id;
    uint16_t rac_ep = room_air_conditioner_endpoint_id;
    ESP_LOGI(TAG, "冷氣電源狀態 → %s", power_on ? "ON" : "OFF");

    /* 驅動 WS2812B 狀態燈：冷氣運轉中顯示紅色，停止時回到已連線綠色 */
    status_led_set_state(power_on ? STATUS_LED_AC_ON : STATUS_LED_CONNECTED);

    chip::DeviceLayer::SystemLayer().ScheduleLambda([contact_ep, rac_ep, power_on]() {
        /* 1. 更新 BooleanState Contact Sensor
         * 語意反轉：AC ON = Open (運轉中需要關注,如門開),AC OFF = Closed。
         * GPIO 邏輯不變 (stable_state=true=ON),只在推給 HomeKit 時反轉。 */
        auto &registry = esp_matter::data_model::provider::get_instance().registry();
        auto *cluster_iface = registry.Get(chip::app::ConcreteClusterPath(contact_ep, BooleanState::Id));
        if (cluster_iface != nullptr) {
            auto *boolean_state = static_cast<BooleanStateCluster *>(cluster_iface);
            boolean_state->SetStateValue(!power_on);  /* AC ON → Open (false) */
        } else {
            ESP_LOGE(TAG, "BooleanState cluster not found for endpoint %d", contact_ep);
        }

        /* 2. 同步 Thermostat + FanMode：GPIO 偵測到的狀態變化不發 IR TX（AC 已實際切換） */
        s_suppress_ir_tx = true;
        if (!power_on) {
            /* AC OFF → SystemMode=Off + FanMode=kOff + PercentCurrent=0 */
            esp_matter_attr_val_t mode_val = esp_matter_attr_val((uint8_t)0, esp_matter_attr_val::uint_sub_type::k_enum);
            attribute::update(rac_ep, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &mode_val);
            esp_matter_attr_val_t fan_val = esp_matter_attr_val((uint8_t)FanControl::FanModeEnum::kOff, esp_matter_attr_val::uint_sub_type::k_enum);
            attribute::update(rac_ep, FanControl::Id, FanControl::Attributes::FanMode::Id, &fan_val);
            {
                lock::ScopedChipStackLock lock(portMAX_DELAY);
                auto *fan_cluster = chip::app::Clusters::FanControl::FindClusterOnEndpoint(rac_ep);
                if (fan_cluster != nullptr) {
                    fan_cluster->SetPercentCurrent(0);
                    ESP_LOGI(TAG, "[gpio] PercentCurrent=0");
                }
            }
        } else {
            /* AC ON → 從 s_ac_state 同步 SystemMode + FanMode + PercentCurrent
             * (AC 可能由遙控器/物理按鍵開機,mode 與上次設的不同) */
            uint8_t matter_mode = 3;  /* Cool 預設 */
            switch (hitachi_ac1_get_mode(&s_ac_state)) {
            case HITACHI_AC1_MODE_COOL: matter_mode = 3; break;
            case HITACHI_AC1_MODE_HEAT: matter_mode = 4; break;
            case HITACHI_AC1_MODE_FAN:
            case HITACHI_AC1_MODE_AUTO: matter_mode = 1; break;
            default: break;
            }
            esp_matter_attr_val_t mode_val = esp_matter_attr_val(matter_mode, esp_matter_attr_val::uint_sub_type::k_enum);
            attribute::update(rac_ep, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &mode_val);

            uint8_t matter_fan = hitachi_fan_to_matter(hitachi_ac1_get_fan(&s_ac_state));
            esp_matter_attr_val_t fan_val = esp_matter_attr_val(matter_fan, esp_matter_attr_val::uint_sub_type::k_enum);
            attribute::update(rac_ep, FanControl::Id, FanControl::Attributes::FanMode::Id, &fan_val);
            {
                lock::ScopedChipStackLock lock(portMAX_DELAY);
                auto *fan_cluster = chip::app::Clusters::FanControl::FindClusterOnEndpoint(rac_ep);
                if (fan_cluster != nullptr) {
                    fan_cluster->SetPercentCurrent(fan_mode_to_percent(matter_fan));
                    ESP_LOGI(TAG, "[gpio] PercentCurrent=%u", (unsigned)fan_mode_to_percent(matter_fan));
                }
            }
        }
        s_suppress_ir_tx = false;
    });
}

/**
 * 把壓縮機狀態推到 contact_sensor endpoint 的 boolean_state.state_value
 * 語意反轉（與 AC power 一致）：運轉=Open,停止=Closed。
 */
void app_driver_update_compressor_state(bool running)
{
    uint16_t endpoint_id = compressor_contact_endpoint_id;
    ESP_LOGI(TAG, "壓縮機運轉狀態 → %s", running ? "RUNNING" : "STOPPED");

    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, running]() {
        auto &registry = esp_matter::data_model::provider::get_instance().registry();
        auto *cluster_iface = registry.Get(chip::app::ConcreteClusterPath(endpoint_id, BooleanState::Id));
        if (cluster_iface == nullptr) {
            ESP_LOGE(TAG, "BooleanState cluster not found for endpoint %d", endpoint_id);
            return;
        }
        auto *boolean_state = static_cast<BooleanStateCluster *>(cluster_iface);
        boolean_state->SetStateValue(!running);  /* 運轉=Open (false) */
    });
}

/**
 * 把 DS18B20 室溫推到 Endpoint 1 的：
 *   1. TemperatureMeasurement.measured_value
 *   2. Thermostat.local_temperature
 * 由 DS18B20 讀取 task 呼叫，需 ScheduleLambda 切到 Matter thread
 *
 * 注意：TemperatureMeasurement 是 codegen cluster (有 CodegenIntegration.h)，
 * 其 attribute storage 由 codegen cluster instance 管理，與 esp-matter storage 不同步。
 * `attribute::update` 對 non-writable attribute 走 set_val_internal 寫 esp-matter storage，
 * 但 `attribute::get_val` / controller read 走 ReadAttribute → data model provider → codegen storage，
 * 兩邊不同步會導致讀到 null。必須用 codegen API `SetMeasuredValue()` 直接寫 codegen storage。
 *
 * Thermostat 是 legacy cluster（沒 CodegenIntegration），attribute 讀寫都走 esp-matter storage，
 * `attribute::update` 可正常使用。
 *
 * 區分方式：看 cluster 的 attribute flags —
 *   - FanMode (FanControl): ATTRIBUTE_FLAG_WRITABLE → attribute::update 走 WriteAttribute → codegen ✓
 *   - MeasuredValue (TemperatureMeasurement): ATTRIBUTE_FLAG_NULLABLE (無 WRITABLE) → 不同步 ✗
 *   - LocalTemperature (Thermostat, legacy): ATTRIBUTE_FLAG_NULLABLE → esp-matter storage (讀寫同源) ✓
 */
void app_driver_update_temperature(float temp_c)
{
    int16_t temp_centdeg = static_cast<int16_t>(temp_c * 100);  /* 0.01°C */
    uint16_t ep = room_air_conditioner_endpoint_id;

    ESP_LOGI(TAG, "室溫更新 → %.2f°C (=%d centi-deg)", temp_c, temp_centdeg);

    chip::DeviceLayer::SystemLayer().ScheduleLambda([ep, temp_centdeg]() {
        /* 1. TemperatureMeasurement.measured_value (codegen cluster — 用 SetMeasuredValue) */
        CHIP_ERROR err = chip::app::Clusters::TemperatureMeasurement::SetMeasuredValue(
            ep, chip::app::DataModel::MakeNullable(temp_centdeg));
        if (err != CHIP_NO_ERROR) {
            ESP_LOGE(TAG, "SetMeasuredValue failed for endpoint %d: %s", ep, err.AsString());
        }

        /* 2. Thermostat.local_temperature (legacy cluster — 用 attribute::update) */
        attribute_t *attr = attribute::get(ep, Thermostat::Id, Thermostat::Attributes::LocalTemperature::Id);
        if (attr != nullptr) {
            esp_matter_attr_val_t val;
            attribute::get_val(attr, &val);
            val.val.i16 = temp_centdeg;
            attribute::update(ep, Thermostat::Id, Thermostat::Attributes::LocalTemperature::Id, &val);
        } else {
            ESP_LOGE(TAG, "Thermostat LocalTemperature not found for endpoint %d", ep);
        }
    });
}

/* ========== 註冊硬體狀態 callback（app_main 呼叫） ========== */
void app_driver_register_callbacks(void)
{
    ds18b20_register_update_cb(app_driver_update_temperature);
    gpio_monitor_register_ac_power_cb(app_driver_update_ac_power_state);
    gpio_monitor_register_compressor_cb(app_driver_update_compressor_state);
    ESP_LOGI(TAG, "Hardware state callbacks registered (DS18B20, AC power, compressor)");

    /* 補觸發目前 GPIO 穩定狀態（解決 callback 註冊前 GPIO monitor 已偵測初始狀態的時序問題）
     * 這樣 LED 與 Matter attribute 都能反映開機時的實際冷氣/壓縮機狀態 */
    gpio_monitor_trigger_current_state();
}

/* ========== contact_set console 指令：手動測試 BooleanState 回報 ========== */
/* 用途：在不依賴 GPIO/實體冷氣的情況下，直接呼叫 SetStateValue 並透過
 *   NotifyAttributeChanged 觸發 attribute report，驗證 HomeKit 是否收到更新。
 * 語法：matter esp contact_set <endpoint 2|3> <0|1>
 * 注意：此指令直接呼叫 SetStateValue(rawValue)，不套用「AC ON=Open」反轉。
 *   raw=1 → HomeKit 顯示 Closed,raw=0 → HomeKit 顯示 Open。 */
static esp_err_t contact_set_handler(int argc, char **argv)
{
    if (argc < 2) {
        ESP_LOGE(TAG, "Usage: matter esp contact_set <endpoint 2|3> <0|1>");
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t ep = (uint16_t)atoi(argv[0]);
    bool val = (atoi(argv[1]) != 0);

    chip::DeviceLayer::SystemLayer().ScheduleLambda([ep, val]() {
        auto &registry = esp_matter::data_model::provider::get_instance().registry();
        auto *cluster_iface = registry.Get(chip::app::ConcreteClusterPath(ep, BooleanState::Id));
        if (cluster_iface == nullptr) {
            ESP_LOGE(TAG, "BooleanState cluster not found for endpoint %d", ep);
            return;
        }
        auto *bs = static_cast<BooleanStateCluster *>(cluster_iface);
        auto ev = bs->SetStateValue(val);
        ESP_LOGI(TAG, "SetStateValue(ep=%u, %d) → event=%s", ep, val, ev.has_value() ? "generated" : "no-change");
    });
    return ESP_OK;
}

namespace esp_matter {
namespace console {
static const command_t contact_commands[] = {
    {
        .name = "contact_set",
        .description = "手動設定 BooleanState StateValue 並觸發 report。Usage: matter esp contact_set <endpoint 2|3> <0|1>",
        .handler = contact_set_handler,
    },
};
} // namespace console
} // namespace esp_matter

esp_err_t app_driver_console_init(void)
{
    return esp_matter::console::add_commands(esp_matter::console::contact_commands,
                                             sizeof(esp_matter::console::contact_commands) / sizeof(esp_matter::console::command_t));
}
