/*
 * Hitachi AC MITM - ir_driver.cpp
 *
 * RMT-based IR 收發 driver，移植自 ESPHome remote_transmitter / remote_receiver
 *
 * 接收：GPIO4 (VS1838B，active-low，invert_in=true)
 *   - RMT RX 解出 mark/space 時長（us），存入 ring buffer 供解碼
 *   - 收到後 log（對應 ESPHome on_raw + logger.log）
 *
 * 發射：GPIO5 (IR LED，38kHz 50% duty，硬體載波 data-phase-only)
 *   - ir_transmit_raw() 把 us timing 轉成 RMT symbol 送出
 *   - 硬體 carrier 在 mark（level=1）自動調變 38kHz，space/idle 自動關閉
 *   - app_driver 會呼叫此函式發射開/關機編碼
 */

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app_priv.h"
#include <esp_matter_console.h>
#include "ir_hitachi_ac1.h"
#include "soc/rmt_struct.h"

static const char *TAG = "ir_driver";

/* IR loopback / compare 測試用的靜態緩衝（避免 stack overflow） */
static uint32_t s_loopback_tx_buf[IR_RAW_MAX_TIMINGS];
static uint32_t s_loopback_rx_buf[IR_RAW_MAX_TIMINGS];

/* RMT channel handles（前方宣告，供 console handler 使用） */
static rmt_channel_handle_t s_tx_channel = NULL;
static rmt_channel_handle_t s_rx_channel = NULL;
static rmt_encoder_handle_t s_copy_encoder = NULL;

/* Debug: 保存最近一批 raw symbols 供 ir_dump 印出（ISR 寫入，task 讀取） */
static rmt_symbol_word_t s_debug_symbols[256];
static size_t s_debug_symbol_count = 0;
static volatile bool s_debug_capture = false;  /* task 設 true 要求 callback 捕獲下一批 */

/* 前向宣告（console handler 在定義之前使用） */
void ir_rx_set_exclusive(bool exclusive);
bool ir_rx_is_exclusive(void);

/* ========== IR LED 測試 console command ========== */
static esp_err_t ir_test_handler(int argc, char **argv)
{
    int count = 1;
    if (argc >= 1) {
        count = atoi(argv[0]);
        if (count <= 0) count = 1;
    }

    ESP_LOGI(TAG, "IR LED 測試：發射 %d 次 NEC 前導脈衝", count);

    /* NEC 前導：9ms mark + 4.5ms space = 13.5ms */
    uint32_t test_pattern[] = {9000, 4500, 560, 560, 560, 560, 560, 1690};

    for (int i = 0; i < count; i++) {
        esp_err_t err = ir_transmit_raw(test_pattern, 8);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "IR 發射失敗：%s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "IR 發射 #%d 完成（用手機相機看 GPIO%d）", i + 1, IR_TRANSMITTER_GPIO);
        }
        vTaskDelay(pdMS_TO_TICKS(500));  /* 間隔 500ms */
    }

    ESP_LOGI(TAG, "IR LED 測試完畢");
    return ESP_OK;
}

/* ========== IR carrier 測試：發射連續 38kHz 載波 ==========
 * 用於驗證 RMT 硬體載波調變是否正常工作。
 * 發射 65ms 連續 mark（uint16_t 最大值），若載波正常，
 * IR receiver 應偵測到 ~65000us 的長 mark。
 * 若只收到極短脈衝（< 1ms），表示載波調變可能未正常運作。
 */
static esp_err_t ir_carrier_handler(int argc, char **argv)
{
    int mode = 0;  /* 0=normal carrier, 1=always_on carrier, 2=no carrier */
    if (argc >= 1) {
        mode = atoi(argv[0]);
    }

    ESP_LOGI(TAG, "=== IR Carrier 測試 (mode=%d: 0=normal, 1=always_on, 2=no carrier) ===", mode);

    /* 動態切換 carrier 設定 */
    rmt_carrier_config_t cfg = {
        .frequency_hz = IR_CARRIER_FREQ_HZ,
        .duty_cycle = (IR_CARRIER_DUTY_PCT / 100.0f),
        .flags = {
            .polarity_active_low = 0,
            .always_on = (mode == 1) ? 1u : 0u,
        },
    };
    if (mode == 2) {
        /* no carrier: 設頻率 0 關閉載波 */
        cfg.frequency_hz = 0;
    }
    esp_err_t err = rmt_apply_carrier(s_tx_channel, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_apply_carrier failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Carrier mode %d applied (freq=%d, always_on=%d)",
             mode, cfg.frequency_hz, cfg.flags.always_on);

    /* 診斷：讀回 register 確認 carrier 設定是否生效 */
    {
        int tx_ch = 1;  /* 我們的 TX channel = ch1 */
        ESP_LOGI(TAG, "[reg after apply] ch%d: carrier_en=%d carrier_eff_en=%d carrier_out_lv=%d",
                 tx_ch, (int)RMT.chnconf0[tx_ch].carrier_en_chn,
                 (int)RMT.chnconf0[tx_ch].carrier_eff_en_chn,
                 (int)RMT.chnconf0[tx_ch].carrier_out_lv_chn);
    }

    ir_rx_clear();

    /* 連續 mark：25000us（< 30ms signal_range_max_ns，避免 premature is_last）
     * + 25000us space（> 20ms gap threshold，觸發 frame_complete 才能讀出 timing） */
    uint32_t carrier_pattern[] = {25000, 25000};

    err = ir_transmit_raw(carrier_pattern, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX 失敗：%s", esp_err_to_name(err));
    }

    /* 等待 RX 處理（callback 在 25ms mark + 25ms space 後觸發 is_last） */
    vTaskDelay(pdMS_TO_TICKS(300));
    ir_rx_flush_stale();

    /* 印 RX 結果。
     * 注意：app_driver_ir_rx_task 每 200ms 輪詢 ir_get_last_received()，
     * 可能搶先消耗掉 s_last_timings。因此主要用 max_accum 判斷成功：
     * max_accum >= 1 表示 RX callback 有收到至少 1 個 symbol（=載波被偵測到） */
    uint32_t cb_count, cb_partial, cb_last, cb_max_accum, cb_rearm;
    size_t last_cb_n;
    ir_rx_get_diag(&cb_count, &cb_partial, &cb_last, &cb_max_accum, &last_cb_n, &cb_rearm);
    ESP_LOGI(TAG, "RX diag: cb=%u (partial=%u last=%u rearm=%u) max_accum=%u last_n=%u",
             (unsigned)cb_count, (unsigned)cb_partial, (unsigned)cb_last,
             (unsigned)cb_rearm, (unsigned)cb_max_accum, (unsigned)last_cb_n);

    /* 主要判斷：max_accum >= 1 表示載波被偵測 */
    if (cb_max_accum >= 1 && last_cb_n >= 1) {
        ESP_LOGI(TAG, ">>> 載波正常：RX 收到 %u symbol(s)，mark 被偵測（last_n=%u）",
                 (unsigned)cb_max_accum, (unsigned)last_cb_n);
    } else {
        ESP_LOGW(TAG, ">>> 載波異常：max_accum=%u last_n=%u（期望 >=1）",
                 (unsigned)cb_max_accum, (unsigned)last_cb_n);
    }

    /* 輔助：嘗試讀取 timing（可能已被 app_driver_ir_rx_task 消耗） */
    size_t rx_count = 0;
    err = ir_get_last_received(s_loopback_rx_buf, IR_RAW_MAX_TIMINGS, &rx_count);
    if (err == ESP_OK && rx_count > 0) {
        ESP_LOGI(TAG, "  (timing 仍可用: %u timings, t[0]=%uus)",
                 (unsigned)rx_count, (unsigned)s_loopback_rx_buf[0]);
    } else {
        ESP_LOGI(TAG, "  (timing 已被 RX task 消耗，用 max_accum 判斷即可)");
    }

    /* 還原：啟用硬體 carrier（預設模式：38kHz data-phase-only） */
    rmt_carrier_config_t restore_cfg = {
        .frequency_hz = IR_CARRIER_FREQ_HZ,
        .duty_cycle = (IR_CARRIER_DUTY_PCT / 100.0f),
        .flags = { .polarity_active_low = 0, .always_on = 0 },
    };
    rmt_apply_carrier(s_tx_channel, &restore_cfg);
    ESP_LOGI(TAG, "Carrier restored to hardware mode (38kHz @ 50%% duty, data-phase-only)");

    return ESP_OK;
}

/* ========== IR compare 測試：用原裝遙控器當基準 ==========
 * 流程：
 * 1. 等待使用者按遙控器（捕獲 RX timings）
 * 2. 解碼成 state
 * 3. 用相同 state 重新 encode 成 TX timings
 * 4. 逐筆比對 RX（遙控器）與 TX（我們的 encoder）的 timing
 * 這樣可以驗證我們的 encoder 產生的波形是否和原裝遙控器一致。
 */
static esp_err_t ir_compare_handler(int argc, char **argv)
{
    ESP_LOGI(TAG, "=== IR Compare 測試 ===");
    ESP_LOGI(TAG, "請按原裝遙控器的任一鍵（15 秒內）...");

    /* 徹底清除 RX 狀態 */
    ir_rx_clear();

    /* 等待接收（最多 15 秒） */
    int wait_ms = 0;
    size_t rx_count = 0;
    while (wait_ms < 15000) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_ms += 100;
        ir_rx_flush_stale();
        if (ir_get_last_received(s_loopback_rx_buf, IR_RAW_MAX_TIMINGS, &rx_count) == ESP_OK) {
            break;
        }
    }

    if (rx_count == 0) {
        ESP_LOGE(TAG, "未收到遙控器訊號");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "收到遙控器訊號：%u timings", (unsigned)rx_count);

    /* 解碼遙控器訊號 */
    hitachi_ac1_state_t remote_state = {};
    esp_err_t err = hitachi_ac1_decode(s_loopback_rx_buf, rx_count, &remote_state);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "遙控器訊號解碼失敗：%s (count=%u)", esp_err_to_name(err), (unsigned)rx_count);
        ESP_LOGI(TAG, "前 32 個 RX timing:");
        for (size_t i = 0; i < rx_count && i < 32; i++) {
            ESP_LOGI(TAG, "  [%u] %u us", (unsigned)i, (unsigned)s_loopback_rx_buf[i]);
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "遙控器 state:");
    hitachi_ac1_state_log(&remote_state);

    /* 用相同 state 重新 encode */
    size_t tx_n = 0;
    err = hitachi_ac1_encode(&remote_state, s_loopback_tx_buf,
                             sizeof(s_loopback_tx_buf) / sizeof(uint32_t), &tx_n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "encode 失敗：%s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "重新 encode：%u timings", (unsigned)tx_n);

    /* 比對 timing 數量 */
    if (rx_count != tx_n) {
        ESP_LOGW(TAG, "timing 數量不符：遙控器=%u encode=%u", (unsigned)rx_count, (unsigned)tx_n);
    } else {
        ESP_LOGI(TAG, "timing 數量一致：%u", (unsigned)tx_n);
    }

    /* 逐筆比對 timing（容差 ±100us） */
    size_t match_count = 0;
    size_t mismatch_count = 0;
    size_t compare_count = (rx_count < tx_n) ? rx_count : tx_n;
    int first_mismatch = -1;

    for (size_t i = 0; i < compare_count; i++) {
        int32_t diff = (int32_t)s_loopback_rx_buf[i] - (int32_t)s_loopback_tx_buf[i];
        if (diff < 0) diff = -diff;
        if (diff <= 100) {
            match_count++;
        } else {
            mismatch_count++;
            if (first_mismatch < 0) {
                first_mismatch = (int)i;
                ESP_LOGW(TAG, "首個不符 timing[%u]：遙控器=%u encode=%u (diff=%d)",
                         (unsigned)i, (unsigned)s_loopback_rx_buf[i],
                         (unsigned)s_loopback_tx_buf[i], diff);
            }
        }
    }

    ESP_LOGI(TAG, "比對結果：%u 一致 / %u 不符 / %u 總數",
             (unsigned)match_count, (unsigned)mismatch_count, (unsigned)compare_count);

    if (mismatch_count == 0 && rx_count == tx_n) {
        ESP_LOGI(TAG, "=== Compare 測試通過：encode 與遙控器訊號完全一致 ===");
    } else if (mismatch_count == 0) {
        ESP_LOGW(TAG, "=== Compare：timing 內容一致但數量不同 ===");
    } else {
        ESP_LOGW(TAG, "=== Compare 測試：有 %u 個 timing 不符 ===", (unsigned)mismatch_count);
        /* dump 不符的 timing 附近 5 個 */
        if (first_mismatch >= 0) {
            size_t start = (first_mismatch > 2) ? first_mismatch - 2 : 0;
            size_t end = (first_mismatch + 3 < compare_count) ? first_mismatch + 3 : compare_count;
            ESP_LOGI(TAG, "不符處附近 (index %u~%u):", (unsigned)start, (unsigned)(end - 1));
            for (size_t i = start; i < end; i++) {
                int32_t diff = (int32_t)s_loopback_rx_buf[i] - (int32_t)s_loopback_tx_buf[i];
                if (diff < 0) diff = -diff;
                ESP_LOGI(TAG, "  [%u] 遙控器=%u encode=%u diff=%d",
                         (unsigned)i, (unsigned)s_loopback_rx_buf[i],
                         (unsigned)s_loopback_tx_buf[i], diff);
            }
        }
    }

    return ESP_OK;
}

/* loopback carrier duty cycle：實測確認 AGC 飽和是 drive capability 過強所致，
 * 與 duty cycle 無關。drive=0 (5mA) + duty=50% 即可穩定接收。
 * 保留可調參數供邊界條件測試。 */
#define IR_LOOPBACK_DUTY_PCT   IR_CARRIER_DUTY_PCT
static void ir_set_carrier_duty_pct(uint8_t pct)
{
    rmt_carrier_config_t cfg = {
        .frequency_hz = IR_CARRIER_FREQ_HZ,
        .duty_cycle = (float)pct / 100.0f,
        .flags = { .polarity_active_low = 0, .always_on = 0 },
    };
    rmt_apply_carrier(s_tx_channel, &cfg);
}

static esp_err_t ir_loopback_handler(int argc, char **argv)
{
    ESP_LOGI(TAG, "=== IR Loopback 測試開始 ===");
    ESP_LOGI(TAG, "請確保 IR LED 指向 IR receiver（< 5cm）");

    /* 可選 duty cycle 參數（1-50），預設 IR_LOOPBACK_DUTY_PCT */
    uint8_t loopback_duty = IR_LOOPBACK_DUTY_PCT;
    if (argc >= 1) {
        int d = atoi(argv[0]);
        if (d >= 1 && d <= 50) {
            loopback_duty = (uint8_t)d;
        } else {
            ESP_LOGE(TAG, "duty_pct 需在 1-50 範圍");
            return ESP_FAIL;
        }
    }

    /* 降低 IR LED 驅動能力避免 VS1838B AGC 飽和。
     * 實測：AGC 飽和由 drive capability 過強（drive=3, 40mA）所致，
     * 與 duty cycle 無關。drive=0 (5mA) + duty=50% 即可穩定 loopback。
     * 預設 IR_LOOPBACK_DUTY_PCT=IR_CARRIER_DUTY_PCT（不改 duty），
     * 可選參數 override duty 供邊界測試。 */
    gpio_drive_cap_t orig_drive;
    gpio_get_drive_capability(IR_TRANSMITTER_GPIO, &orig_drive);
    gpio_set_drive_capability(IR_TRANSMITTER_GPIO, GPIO_DRIVE_CAP_0);
    ESP_LOGI(TAG, "Drive capability: %d → 0 (loopback mode)", (int)orig_drive);

    /* carrier duty cycle：預設與正常傳送相同（IR_CARRIER_DUTY_PCT）；
     * 可用 `ir_loopback <duty_pct>` override 供邊界測試 */
    ir_set_carrier_duty_pct(loopback_duty);
    ESP_LOGI(TAG, "Carrier duty: %d%% → %d%% (loopback mode)", (int)IR_CARRIER_DUTY_PCT, (int)loopback_duty);

    /* 構造測試 state：Power=ON, Mode=COOL, Temp=24°C */
    hitachi_ac1_state_t tx_state = {};
    hitachi_ac1_state_reset(&tx_state);  /* 設定固定 header + 預設值 */
    hitachi_ac1_set_model(&tx_state, HITACHI_AC1_MODEL_B);
    hitachi_ac1_set_power(&tx_state, true);
    hitachi_ac1_set_mode(&tx_state, HITACHI_AC1_MODE_COOL);
    hitachi_ac1_set_temp(&tx_state, 24);

    /* 編碼 */
    size_t tx_n = 0;
    esp_err_t err = hitachi_ac1_encode(&tx_state, s_loopback_tx_buf,
                                        sizeof(s_loopback_tx_buf) / sizeof(uint32_t), &tx_n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "encode 失敗：%s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "發射 state:");
    hitachi_ac1_state_log(&tx_state);
    ESP_LOGI(TAG, "TX: %u timings", (unsigned)tx_n);

    /* 確保 RX 硬體已啟動（loopback 不停止 RX — 需接收自己的 TX）
     * 若 app_driver_ir_send_state 的 confirm task 尚未 resume，RX 會停擺 */
    ir_rx_start();

    /* 徹底清除 RX 狀態（累積緩衝、last_timings、診斷計數） */
    ir_rx_clear();

    /* 獨佔 RX：防止 app_driver task 讀取並清除 s_last_count */
    ir_rx_set_exclusive(true);

    /* 發射 */
    err = ir_transmit_raw(s_loopback_tx_buf, tx_n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "發射失敗：%s", esp_err_to_name(err));
        ir_rx_set_exclusive(false);
        ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
        gpio_set_drive_capability(IR_TRANSMITTER_GPIO, orig_drive);
        return ESP_FAIL;
    }

    /* 等待接收。
     * TX 約 150ms + RMT idle 偵測 30ms + flush 超時 150ms ≈ 330ms。
     * 設 600ms 等待 + 50ms 輪詢，確保 flush 後能取到資料。 */
    int wait_ms = 0;
    size_t rx_count = 0;
    while (wait_ms < 600) {
        vTaskDelay(pdMS_TO_TICKS(50));
        wait_ms += 50;
        ir_rx_flush_stale();
        if (ir_get_last_received(s_loopback_rx_buf, IR_RAW_MAX_TIMINGS, &rx_count) == ESP_OK) {
            break;
        }
    }

    /* 釋放 RX 獨佔 */
    ir_rx_set_exclusive(false);

    /* 印診斷 */
    uint32_t cb_count, cb_partial, cb_last, cb_max_accum, cb_rearm;
    size_t last_cb_n;
    ir_rx_get_diag(&cb_count, &cb_partial, &cb_last, &cb_max_accum, &last_cb_n, &cb_rearm);
    ESP_LOGI(TAG, "RX diag: cb=%u partial=%u last=%u rearm=%u max_accum=%u last_n=%u",
             (unsigned)cb_count, (unsigned)cb_partial, (unsigned)cb_last,
             (unsigned)cb_rearm, (unsigned)cb_max_accum, (unsigned)last_cb_n);

    if (rx_count == 0) {
        ESP_LOGE(TAG, "未收到訊號（IR LED 太弱或未對準）");
        ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
        gpio_set_drive_capability(IR_TRANSMITTER_GPIO, orig_drive);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RX: %u timings", (unsigned)rx_count);

    /* HITACHI_AC1 encode 輸出 212 timings（含最後 100ms GAP），
     * 但 RX 不會收到 GAP（它是訊框結束標記），所以 RX=211 是正確的。
     * 這裡只檢查 RX 數量是否在合理範圍（HITACHI_AC1 = 211） */
    if (rx_count == tx_n - 1) {
        ESP_LOGI(TAG, "timing 數量正確：TX=%u RX=%u（最後一筆 TX 是 GAP，RX 不含）",
                 (unsigned)tx_n, (unsigned)rx_count);
    } else if (rx_count != tx_n) {
        ESP_LOGW(TAG, "timing 數量不符：TX=%u RX=%u", (unsigned)tx_n, (unsigned)rx_count);
    } else {
        ESP_LOGI(TAG, "timing 數量一致：%u", (unsigned)tx_n);
    }

    /* 解碼接收到的訊號 */
    hitachi_ac1_state_t rx_state = {};
    err = hitachi_ac1_decode(s_loopback_rx_buf, rx_count, &rx_state);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "解碼失敗：%s (count=%u)", esp_err_to_name(err), (unsigned)rx_count);
        /* dump 所有 RX timing 供分析 */
        for (size_t i = 0; i < rx_count; i++) {
            ESP_LOGI(TAG, "  [%u] %u us %s", (unsigned)i, (unsigned)s_loopback_rx_buf[i],
                     (i % 2 == 0) ? "(mark)" : "(space)");
        }
        gpio_set_drive_capability(IR_TRANSMITTER_GPIO, orig_drive);
        ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
        ESP_LOGI(TAG, "Drive capability restored to %d, carrier duty restored to %d%%",
                 (int)orig_drive, (int)IR_CARRIER_DUTY_PCT);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "接收 state:");
    hitachi_ac1_state_log(&rx_state);

    /* 比對 */
    bool match = true;
    if (hitachi_ac1_get_power(&tx_state) != hitachi_ac1_get_power(&rx_state)) {
        ESP_LOGW(TAG, "Power 不符：TX=%d RX=%d",
                 hitachi_ac1_get_power(&tx_state), hitachi_ac1_get_power(&rx_state));
        match = false;
    }
    if (hitachi_ac1_get_temp(&tx_state) != hitachi_ac1_get_temp(&rx_state)) {
        ESP_LOGW(TAG, "Temp 不符：TX=%d RX=%d",
                 hitachi_ac1_get_temp(&tx_state), hitachi_ac1_get_temp(&rx_state));
        match = false;
    }
    if (hitachi_ac1_get_mode(&tx_state) != hitachi_ac1_get_mode(&rx_state)) {
        ESP_LOGW(TAG, "Mode 不符：TX=%d RX=%d",
                 hitachi_ac1_get_mode(&tx_state), hitachi_ac1_get_mode(&rx_state));
        match = false;
    }

    if (match) {
        ESP_LOGI(TAG, "=== Loopback 測試通過：發射與接收 state 一致 ===");
    } else {
        ESP_LOGW(TAG, "=== Loopback 測試：state 不一致（請檢查） ===");
    }

    /* 恢復 carrier duty cycle 與 IR LED 驅動能力 */
    ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
    gpio_set_drive_capability(IR_TRANSMITTER_GPIO, orig_drive);
    ESP_LOGI(TAG, "Drive capability restored to %d, carrier duty restored to %d%%",
             (int)orig_drive, (int)IR_CARRIER_DUTY_PCT);

    return ESP_OK;
}

/* ========== IR listen：持續監聽並印出所有接收到的 IR 訊號 ==========
 * 用於診斷 IR receiver 是否正常接收。即使收到的訊號不完整也會印出。
 * 預設監聽 30 秒，可用參數指定秒數（最多 120 秒）。
 */
static esp_err_t ir_listen_handler(int argc, char **argv)
{
    int duration_sec = 30;
    if (argc >= 1) {
        duration_sec = atoi(argv[0]);
        if (duration_sec <= 0) duration_sec = 30;
        if (duration_sec > 120) duration_sec = 120;
    }

    ESP_LOGI(TAG, "=== IR Listen（持續 %d 秒）===", duration_sec);
    ESP_LOGI(TAG, "對著 IR receiver 按遙控器，所有收到的訊號都會印出");

    /* GPIO4 raw level 診斷：idle 應為 HIGH（VS1838B active-low + 上拉） */
    int gpio_idle = gpio_get_level(IR_RECEIVER_GPIO);
    ESP_LOGI(TAG, "GPIO%d idle level = %d (expect 1=HIGH; 0=LOW 可能短路或接線錯誤)",
             IR_RECEIVER_GPIO, gpio_idle);

    ir_rx_clear();

    /* 獨佔 RX：防止 app_driver task 讀取並清除 s_last_count */
    ir_rx_set_exclusive(true);

    uint32_t last_cb = 0;
    uint32_t frame_num = 0;
    int elapsed_ms = 0;

    while (elapsed_ms < duration_sec * 1000) {
        vTaskDelay(pdMS_TO_TICKS(50));
        elapsed_ms += 50;

        /* 印診斷變化 */
        uint32_t cb_count, cb_partial, cb_last, cb_max_accum, cb_rearm;
        size_t last_cb_n;
        ir_rx_get_diag(&cb_count, &cb_partial, &cb_last, &cb_max_accum, &last_cb_n, &cb_rearm);
        if (cb_count != last_cb) {
            last_cb = cb_count;
            ESP_LOGI(TAG, "[diag] cb=%u (partial=%u last=%u rearm=%u) max_accum=%u last_n=%u",
                     (unsigned)cb_count, (unsigned)cb_partial, (unsigned)cb_last,
                     (unsigned)cb_rearm, (unsigned)cb_max_accum, (unsigned)last_cb_n);
        }

        /* 超時 flush（讓不完整的訊框也能被取出） */
        ir_rx_flush_stale();

        /* 取出任何已接收的訊框 */
        size_t rx_count = 0;
        esp_err_t err = ir_get_last_received(s_loopback_rx_buf, IR_RAW_MAX_TIMINGS, &rx_count);
        if (err == ESP_OK && rx_count > 0) {
            frame_num++;
            ESP_LOGI(TAG, "--- Frame #%u: %u timings ---", (unsigned)frame_num, (unsigned)rx_count);

            /* 嘗試解碼 */
            hitachi_ac1_state_t state = {};
            err = hitachi_ac1_decode(s_loopback_rx_buf, rx_count, &state);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "解碼成功:");
                hitachi_ac1_state_log(&state);
            } else {
                ESP_LOGW(TAG, "解碼失敗: %s（可能訊號不完整）", esp_err_to_name(err));
            }

            /* 印前 32 個 timing 供分析 */
            size_t dump_n = (rx_count < 32) ? rx_count : 32;
            for (size_t i = 0; i < dump_n; i++) {
                ESP_LOGI(TAG, "  t[%u]=%u", (unsigned)i, (unsigned)s_loopback_rx_buf[i]);
            }
            if (rx_count > 32) {
                ESP_LOGI(TAG, "  ... (共 %u timings)", (unsigned)rx_count);
            }
        }
    }

    ESP_LOGI(TAG, "=== IR Listen 結束（共 %u 個訊框）===", (unsigned)frame_num);
    ir_rx_set_exclusive(false);
    return ESP_OK;
}

/* ========== IR roundtrip 測試：encode → decode（純軟體，不需 IR LED） ==========
 * 驗證 encoder 產生的 timings 可以被 decoder 正確解回相同 state。
 * 這是 loopback 測試的軟體版本，排除 IR LED 硬體問題。
 */
static esp_err_t ir_roundtrip_handler(int argc, char **argv)
{
    ESP_LOGI(TAG, "=== IR Roundtrip 測試（純軟體 encode→decode） ===");

    /* 構造測試 state */
    hitachi_ac1_state_t tx_state = {};
    hitachi_ac1_state_reset(&tx_state);
    hitachi_ac1_set_model(&tx_state, HITACHI_AC1_MODEL_A);
    hitachi_ac1_set_power(&tx_state, true);
    hitachi_ac1_set_mode(&tx_state, HITACHI_AC1_MODE_COOL);
    hitachi_ac1_set_temp(&tx_state, 24);
    hitachi_ac1_set_fan(&tx_state, HITACHI_AC1_FAN_LOW);

    ESP_LOGI(TAG, "原始 state:");
    hitachi_ac1_state_log(&tx_state);

    /* 編碼 */
    size_t tx_n = 0;
    esp_err_t err = hitachi_ac1_encode(&tx_state, s_loopback_tx_buf,
                                        sizeof(s_loopback_tx_buf) / sizeof(uint32_t), &tx_n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "encode 失敗：%s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "encode: %u timings", (unsigned)tx_n);

    /* 印前 10 個 timing */
    for (size_t i = 0; i < tx_n && i < 10; i++) {
        ESP_LOGI(TAG, "  timing[%u] = %u us", (unsigned)i, (unsigned)s_loopback_tx_buf[i]);
    }

    /* 直接解碼（不經過 IR LED / RX） */
    hitachi_ac1_state_t rx_state = {};
    err = hitachi_ac1_decode(s_loopback_tx_buf, tx_n, &rx_state);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "decode 失敗：%s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "解碼 state:");
    hitachi_ac1_state_log(&rx_state);

    /* 比對 raw bytes */
    bool match = true;
    for (int i = 0; i < HITACHI_AC1_STATE_LEN; i++) {
        if (tx_state.raw[i] != rx_state.raw[i]) {
            ESP_LOGW(TAG, "Byte %d 不符：TX=0x%02X RX=0x%02X",
                     i, tx_state.raw[i], rx_state.raw[i]);
            match = false;
        }
    }

    if (match) {
        ESP_LOGI(TAG, "=== Roundtrip 通過：encode→decode state 完全一致 ===");
    } else {
        ESP_LOGW(TAG, "=== Roundback 失敗：state 不一致 ===");
    }

    return ESP_OK;
}

/* ========== ir_dump：印出 RMT RX callback 收到的 raw symbols ==========
 * 用於診斷 RX 是否收到完整的 mark/space pair，或是只收到 header mark。
 * 與 ir_loopback 類似，但直接印 rmt_symbol_word_t 的 duration0/duration1，
 * 不經過 flatten 邏輯，方便看出 RMT 硬體到底回報什麼。
 *
 * 用法：matter esp ir_dump
 * 結果會印出 ISR callback 捕獲的第一批 symbols（最多 16 個）的
 *   [i] level0=... duration0=Nus  level1=... duration1=Mus
 */
static esp_err_t ir_dump_handler(int argc, char **argv)
{
    ESP_LOGI(TAG, "=== IR Dump：發射測試訊號並印出 RMT raw symbols ===");
    ESP_LOGI(TAG, "請將 IR LED 指向 IR receiver (< 5cm)");

    /* 降低驅動能力與 carrier duty cycle 避免 AGC 飽和 */
    gpio_drive_cap_t orig_drive;
    gpio_get_drive_capability(IR_TRANSMITTER_GPIO, &orig_drive);
    gpio_set_drive_capability(IR_TRANSMITTER_GPIO, GPIO_DRIVE_CAP_0);
    ir_set_carrier_duty_pct(IR_LOOPBACK_DUTY_PCT);

    /* 構造測試 state */
    hitachi_ac1_state_t tx_state = {};
    hitachi_ac1_state_reset(&tx_state);
    hitachi_ac1_set_model(&tx_state, HITACHI_AC1_MODEL_B);
    hitachi_ac1_set_power(&tx_state, true);
    hitachi_ac1_set_mode(&tx_state, HITACHI_AC1_MODE_COOL);
    hitachi_ac1_set_temp(&tx_state, 24);

    size_t tx_n = 0;
    esp_err_t err = hitachi_ac1_encode(&tx_state, s_loopback_tx_buf,
                                        sizeof(s_loopback_tx_buf) / sizeof(uint32_t), &tx_n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "encode 失敗：%s", esp_err_to_name(err));
        ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
        gpio_set_drive_capability(IR_TRANSMITTER_GPIO, orig_drive);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "TX: %u timings", (unsigned)tx_n);

    /* 清除 RX 狀態並要求 callback 捕獲下一批 symbols */
    ir_rx_clear();
    ir_rx_set_exclusive(true);
    s_debug_symbol_count = 0;
    s_debug_capture = true;

    /* 發射 */
    err = ir_transmit_raw(s_loopback_tx_buf, tx_n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "發射失敗：%s", esp_err_to_name(err));
        ir_rx_set_exclusive(false);
        ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
        gpio_set_drive_capability(IR_TRANSMITTER_GPIO, orig_drive);
        return ESP_FAIL;
    }

    /* 等 callback 觸發（最多 500ms） */
    for (int i = 0; i < 50 && s_debug_capture; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (s_debug_capture) {
        ESP_LOGE(TAG, "未收到任何 callback（500ms 超時）");
        s_debug_capture = false;
        ir_rx_set_exclusive(false);
        ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
        gpio_set_drive_capability(IR_TRANSMITTER_GPIO, orig_drive);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "callback 捕獲 %u 個 raw symbol：", (unsigned)s_debug_symbol_count);
    for (size_t i = 0; i < s_debug_symbol_count; i++) {
        uint32_t d0 = s_debug_symbols[i].duration0;
        uint32_t d1 = s_debug_symbols[i].duration1;
        uint32_t l0 = s_debug_symbols[i].level0;
        uint32_t l1 = s_debug_symbols[i].level1;
        ESP_LOGI(TAG, "  sym[%u] lv0=%u d0=%uus  lv1=%u d1=%uus",
                 (unsigned)i, (unsigned)l0, (unsigned)d0,
                 (unsigned)l1, (unsigned)d1);
    }

    /* 對照：印出 TX 期望的前幾個 timings */
    ESP_LOGI(TAG, "對照 TX 期望（前 8 個 timings）：");
    for (size_t i = 0; i < 8 && i < tx_n; i++) {
        ESP_LOGI(TAG, "  tx[%u]=%uus %s", (unsigned)i, (unsigned)s_loopback_tx_buf[i],
                 (i % 2 == 0) ? "(mark)" : "(space)");
    }

    ir_rx_set_exclusive(false);
    ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
    gpio_set_drive_capability(IR_TRANSMITTER_GPIO, orig_drive);
    return ESP_OK;
}

/* ========== ir_send：發射 HITACHI_AC1 state 給真實冷氣 ==========
 * 不嘗試接收（VS1838B 對短 pulse 靈敏度不足，loopback 不可靠）。
 * 直接發射 IR 訊號，靠冷氣本身的 IR receiver（靈敏度遠高於 VS1838B）接收。
 * 用法：
 *   matter esp ir_send              # 預設：COOL 24°C 開機
 *   matter esp ir_send off           # 關機
 *   matter esp ir_send on 26 heat    # 開機 26°C 暖氣
 *   matter esp ir_send on 24 cool 3  # 開機 24°C 冷氣, 重複發射 3 次
 */
static esp_err_t ir_send_handler(int argc, char **argv)
{
    hitachi_ac1_state_t state = {};
    hitachi_ac1_state_reset(&state);
    hitachi_ac1_set_model(&state, HITACHI_AC1_MODEL_B);

    bool power = true;
    uint8_t temp = 24;
    uint8_t ac_mode = HITACHI_AC1_MODE_COOL;
    int repeat = 1;  /* 預設發射 1 次 */

    /* 解析參數 */
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "on") == 0) {
            power = true;
        } else if (strcmp(argv[i], "off") == 0) {
            power = false;
        } else if (strcmp(argv[i], "cool") == 0) {
            ac_mode = HITACHI_AC1_MODE_COOL;
        } else if (strcmp(argv[i], "heat") == 0) {
            ac_mode = HITACHI_AC1_MODE_HEAT;
        } else if (strcmp(argv[i], "fan") == 0) {
            ac_mode = HITACHI_AC1_MODE_FAN;
        } else if (strcmp(argv[i], "dry") == 0) {
            ac_mode = HITACHI_AC1_MODE_DRY;
        } else if (strcmp(argv[i], "auto") == 0) {
            ac_mode = HITACHI_AC1_MODE_AUTO;
        } else {
            /* 嘗試當作數值：16-31 = 溫度, 2-15 = 重複次數 */
            int t = atoi(argv[i]);
            if (t >= 16 && t <= 31) {
                temp = (uint8_t)t;
            } else if (t >= 2 && t <= 15) {
                repeat = t;
            }
        }
    }

    /* Hitachi AC1 電源是 toggle 模式：PowerToggle=1 即切換,Power bit 不影響。
     * 實測只有 Power=0 的 TX 能讓 AC 回應,故 on/off 都用 Power=0 + PowerToggle=1。
     * AC 收到 PowerToggle=1 就切換電源狀態,與 Power bit 無關。 */
    hitachi_ac1_set_power(&state, false);
    hitachi_ac1_set_power_toggle(&state, true);
    hitachi_ac1_set_mode(&state, ac_mode);
    hitachi_ac1_set_temp(&state, temp);
    hitachi_ac1_set_fan(&state, HITACHI_AC1_FAN_AUTO);

    ESP_LOGI(TAG, "=== IR Send：發射 HITACHI_AC1 state 給冷氣 (repeat=%d, power=%s→toggle) ===",
             repeat, power ? "ON" : "OFF");
    ESP_LOGI(TAG, "請將 IR LED（GPIO%d）指向冷氣接收器（< 1m）", IR_TRANSMITTER_GPIO);
    hitachi_ac1_state_log(&state);

    /* 編碼 */
    size_t n = 0;
    esp_err_t err = hitachi_ac1_encode(&state, s_loopback_tx_buf,
                                        sizeof(s_loopback_tx_buf) / sizeof(uint32_t), &n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "encode 失敗：%s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "TX: %u timings, repeat=%d", (unsigned)n, repeat);

    /* 發射 repeat 次 */
    for (int r = 0; r < repeat; r++) {
        err = ir_transmit_raw(s_loopback_tx_buf, n);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "TX #%d/%d 失敗：%s", r + 1, repeat, esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "IR 發射 #%d/%d 完成", r + 1, repeat);
        if (r < repeat - 1) {
            vTaskDelay(pdMS_TO_TICKS(200));  /* 段間 200ms */
        }
    }

    ESP_LOGI(TAG, "IR 發射完成。請觀察冷氣是否回應（GPIO10 光敏電阻 電源狀態）");
    return ESP_OK;
}

/* ========== ir_replay：捕獲遙控器訊號並原樣重播 ==========
 * 用於診斷「只有 off 有反應」問題：
 *   1. 用原裝遙控器按 ON 鍵 → ESP32 捕獲 exact timings
 *   2. 用遙控器或 ir_send off 關閉冷氣
 *   3. ir_replay 重播捕獲的 ON 訊號
 *   - 若 AC 回應 → IR 硬體正常，問題在我們的 encoder
 *   - 若 AC 不回應 → IR 硬體問題（LED 太弱、距離太遠等）
 *
 * 用法：
 *   matter esp ir_replay           # 捕獲後立即重播 1 次
 *   matter esp ir_replay 3         # 捕獲後重播 3 次
 */
static esp_err_t ir_replay_handler(int argc, char **argv)
{
    int repeat = 1;
    if (argc >= 1) {
        repeat = atoi(argv[0]);
        if (repeat < 1) repeat = 1;
        if (repeat > 10) repeat = 10;
    }

    ESP_LOGI(TAG, "=== IR Replay：捕獲遙控器訊號並原樣重播 (repeat=%d) ===", repeat);
    ESP_LOGI(TAG, "請按原裝遙控器的按鍵（15 秒內）...");

    /* 清除 RX 狀態 */
    ir_rx_clear();

    /* 等待接收 */
    int wait_ms = 0;
    size_t rx_count = 0;
    while (wait_ms < 15000) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_ms += 100;
        ir_rx_flush_stale();
        if (ir_get_last_received(s_loopback_rx_buf, IR_RAW_MAX_TIMINGS, &rx_count) == ESP_OK) {
            break;
        }
    }

    if (rx_count == 0) {
        ESP_LOGE(TAG, "未收到遙控器訊號（15 秒超時）");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "捕獲 %u timings", (unsigned)rx_count);

    /* 嘗試解碼以顯示 state */
    hitachi_ac1_state_t state = {};
    esp_err_t err = hitachi_ac1_decode(s_loopback_rx_buf, rx_count, &state);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "解碼成功:");
        hitachi_ac1_state_log(&state);
    } else {
        ESP_LOGW(TAG, "解碼失敗: %s（仍可重播 raw timings）", esp_err_to_name(err));
    }

    /* 印前 16 個 timing 供參考 */
    for (size_t i = 0; i < rx_count && i < 16; i++) {
        ESP_LOGI(TAG, "  t[%u]=%u us", (unsigned)i, (unsigned)s_loopback_rx_buf[i]);
    }

    ESP_LOGI(TAG, "即將重播 %u timings × %d 次，請將 IR LED 指向冷氣...", (unsigned)rx_count, repeat);
    vTaskDelay(pdMS_TO_TICKS(2000));  /* 2 秒準備時間 */

    /* 原樣重播 */
    for (int r = 0; r < repeat; r++) {
        err = ir_transmit_raw(s_loopback_rx_buf, rx_count);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "重播 #%d/%d 失敗：%s", r + 1, repeat, esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "重播 #%d/%d 完成", r + 1, repeat);
        if (r < repeat - 1) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    ESP_LOGI(TAG, "IR Replay 完成。請觀察冷氣是否回應");
    return ESP_OK;
}

/* ========== ir_reg：讀取 RMT TX/RX register 診斷 ==========
 * TX channel (chn, 0-1)：carrier 設定
 * RX channel (chm, 2-3)：idle threshold, filter, carrier demod */
static esp_err_t ir_reg_handler(int argc, char **argv)
{
    ESP_LOGI(TAG, "=== RMT register dump ===");
    ESP_LOGI(TAG, "--- TX channels (chn 0-1) ---");
    for (int ch = 0; ch < 2; ch++) {
        uint32_t val = RMT.chnconf0[ch].val;
        ESP_LOGI(TAG, "TX ch%d chnconf0=0x%08" PRIx32, ch, val);
        ESP_LOGI(TAG, "  carrier_en=%d carrier_eff_en=%d carrier_out_lv=%d",
                 (int)RMT.chnconf0[ch].carrier_en_chn,
                 (int)RMT.chnconf0[ch].carrier_eff_en_chn,
                 (int)RMT.chnconf0[ch].carrier_out_lv_chn);
        ESP_LOGI(TAG, "  idle_out_en=%d idle_out_lv=%d div_cnt=%d",
                 (int)RMT.chnconf0[ch].idle_out_en_chn,
                 (int)RMT.chnconf0[ch].idle_out_lv_chn,
                 (int)RMT.chnconf0[ch].div_cnt_chn);
        /* carrier duty 暫存器：carrier_high/low 以 group clock 為單位
         * ESP32-C6 RMT_CLK_SRC_DEFAULT = PLL_F80M (80MHz)，group_prescale=1 → group_resolution=80MHz
         * carrier module works base on group clock（rmt_tx.c:887），不是 channel clock
         * 38kHz @ 80MHz group clock → total_ticks = 80M/38k = 2105
         * 50% duty → high=1052 low=1053 */
        uint32_t duty_val = RMT.chncarrier_duty[ch].val;
        uint16_t ch_low = RMT.chncarrier_duty[ch].carrier_low_chn;
        uint16_t ch_high = RMT.chncarrier_duty[ch].carrier_high_chn;
        uint32_t total = (uint32_t)ch_low + ch_high;
        ESP_LOGI(TAG, "  chncarrier_duty=0x%08" PRIx32 " high=%u low=%u (sum=%u)",
                 duty_val, (unsigned)ch_high, (unsigned)ch_low, (unsigned)total);
        if (total > 0) {
            uint32_t duty_pct = (uint32_t)ch_high * 100 / total;
            /* group clock = 80MHz (PLL_F80M, RMT_CLK_SRC_DEFAULT, group_prescale=1)
             * carrier_freq = group_resolution / total_ticks */
            uint32_t freq_hz = 80000000 / total;
            ESP_LOGI(TAG, "  → duty=%u%% freq=%uHz (group_clock=80MHz)", (unsigned)duty_pct, (unsigned)freq_hz);
        }
    }
    ESP_LOGI(TAG, "--- RX channels (chm 0-1) ---");
    for (int ch = 0; ch < 2; ch++) {
        uint32_t val = RMT.chmconf[ch].conf0.val;
        ESP_LOGI(TAG, "RX ch%d chmconf0=0x%08" PRIx32, ch, val);
        ESP_LOGI(TAG, "  idle_thres=%d div_cnt=%d mem_size=%d carrier_en=%d",
                 (int)RMT.chmconf[ch].conf0.idle_thres_chm,
                 (int)RMT.chmconf[ch].conf0.div_cnt_chm,
                 (int)RMT.chmconf[ch].conf0.mem_size_chm,
                 (int)RMT.chmconf[ch].conf0.carrier_en_chm);
        uint32_t conf1 = RMT.chmconf[ch].conf1.val;
        ESP_LOGI(TAG, "RX ch%d chmconf1=0x%08" PRIx32 " rx_en=%d mem_owner=%d rx_filter_en=%d rx_filter_thres=%u (%uus @80MHz)",
                 ch, conf1,
                 (int)RMT.chmconf[ch].conf1.rx_en_chm,
                 (int)RMT.chmconf[ch].conf1.mem_owner_chm,
                 (int)RMT.chmconf[ch].conf1.rx_filter_en_chm,
                 (int)RMT.chmconf[ch].conf1.rx_filter_thres_chm,
                 (unsigned)(RMT.chmconf[ch].conf1.rx_filter_thres_chm / 80));
    }
    gpio_drive_cap_t drv_cap;
    gpio_get_drive_capability(IR_TRANSMITTER_GPIO, &drv_cap);
    ESP_LOGI(TAG, "GPIO5 (IR LED) level=%d drive_cap=%d (0=weakest, 3=strongest)",
             gpio_get_level(IR_TRANSMITTER_GPIO), (int)drv_cap);
    ESP_LOGI(TAG, "GPIO4 (IR RX) level=%d", gpio_get_level(IR_RECEIVER_GPIO));
    return ESP_OK;
}

/* ========== ir_set_drive：設定 IR LED GPIO 驅動能力 ==========
 * 用法：matter esp ir_set_drive [0|1|2|3]
 *   0 = weakest (~5mA, 適合 loopback)
 *   3 = strongest (~40mA, 適合遠距離發射給真實冷氣) */
static esp_err_t ir_set_drive_handler(int argc, char **argv)
{
    int level = 0;
    if (argc >= 1) {
        level = atoi(argv[0]);
    }
    if (level < 0 || level > 3) {
        ESP_LOGE(TAG, "drive level must be 0-3, got %d", level);
        return ESP_ERR_INVALID_ARG;
    }
    gpio_set_drive_capability(IR_TRANSMITTER_GPIO, (gpio_drive_cap_t)level);
    gpio_drive_cap_t drv_cap;
    gpio_get_drive_capability(IR_TRANSMITTER_GPIO, &drv_cap);
    ESP_LOGI(TAG, "IR TX GPIO%d drive capability set to %d", IR_TRANSMITTER_GPIO, (int)drv_cap);
    return ESP_OK;
}

/* ========== ir_pulse：發射單一 mark/space 測試 VS1838B 靈敏度 ==========
 * 用法：matter esp ir_pulse [mark_us] [space_us] [mark2_us] [space2_us] ...
 * 預設：mark=400 space=10000（測試 400us bit mark 是否可被偵測）
 * 不含 header mark，避免 AGC 飽和影響判斷
 * 可接多組 mark/space，例如：ir_pulse 3400 3400 400 10000 測試 AGC 飽和 */
static esp_err_t ir_pulse_handler(int argc, char **argv)
{
    if (argc == 0) {
        ESP_LOGE(TAG, "用法：ir_pulse [mark_us space_us]+");
        return ESP_ERR_INVALID_ARG;
    }
    if (argc % 2 != 0) {
        ESP_LOGE(TAG, "參數必須成對 (mark_us space_us)，收到 %d 個", argc);
        return ESP_ERR_INVALID_ARG;
    }

    size_t n_pairs = argc / 2;
    uint32_t pattern[16];
    if (n_pairs > 8) {
        ESP_LOGE(TAG, "最多 8 組 mark/space");
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < n_pairs; i++) {
        pattern[i*2]   = (uint32_t)atoi(argv[i*2]);
        pattern[i*2+1] = (uint32_t)atoi(argv[i*2+1]);
    }

    ESP_LOGI(TAG, "=== IR Pulse 測試：%u 組 mark/space ===", (unsigned)n_pairs);
    for (size_t i = 0; i < n_pairs; i++) {
        ESP_LOGI(TAG, "  [%u] mark=%uus space=%uus",
                 (unsigned)i, (unsigned)pattern[i*2], (unsigned)pattern[i*2+1]);
    }

    /* 降低驅動能力與 carrier duty cycle */
    gpio_drive_cap_t pulse_orig_drive;
    gpio_get_drive_capability(IR_TRANSMITTER_GPIO, &pulse_orig_drive);
    gpio_set_drive_capability(IR_TRANSMITTER_GPIO, GPIO_DRIVE_CAP_0);
    ir_set_carrier_duty_pct(IR_LOOPBACK_DUTY_PCT);

    ir_rx_clear();
    s_debug_symbol_count = 0;
    s_debug_capture = true;

    esp_err_t err = ir_transmit_raw(pattern, n_pairs * 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX 失敗：%s", esp_err_to_name(err));
        ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
        gpio_set_drive_capability(IR_TRANSMITTER_GPIO, pulse_orig_drive);
        return ESP_FAIL;
    }

    /* 等待所有 callback（包括 re-arm 後的）— 最多 500ms */
    vTaskDelay(pdMS_TO_TICKS(500));

    if (s_debug_capture) {
        ESP_LOGE(TAG, "未收到任何 callback（500ms 超時）— VS1838B 未偵測到載波");
        s_debug_capture = false;
        ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
        gpio_set_drive_capability(IR_TRANSMITTER_GPIO, pulse_orig_drive);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "callback 捕獲 %u 個 raw symbol（期望 %u 個 mark）：",
             (unsigned)s_debug_symbol_count, (unsigned)n_pairs);
    for (size_t i = 0; i < s_debug_symbol_count; i++) {
        uint32_t d0 = s_debug_symbols[i].duration0;
        uint32_t d1 = s_debug_symbols[i].duration1;
        uint32_t l0 = s_debug_symbols[i].level0;
        uint32_t l1 = s_debug_symbols[i].level1;
        ESP_LOGI(TAG, "  sym[%u] lv0=%u d0=%uus  lv1=%u d1=%uus",
                 (unsigned)i, (unsigned)l0, (unsigned)d0,
                 (unsigned)l1, (unsigned)d1);
    }

    uint32_t cb_count, cb_partial, cb_last, cb_max_accum, cb_rearm;
    size_t last_cb_n;
    ir_rx_get_diag(&cb_count, &cb_partial, &cb_last, &cb_max_accum, &last_cb_n, &cb_rearm);
    ESP_LOGI(TAG, "RX diag: cb=%u partial=%u last=%u rearm=%u max_accum=%u last_n=%u",
             (unsigned)cb_count, (unsigned)cb_partial, (unsigned)cb_last,
             (unsigned)cb_rearm, (unsigned)cb_max_accum, (unsigned)last_cb_n);

    ir_set_carrier_duty_pct(IR_CARRIER_DUTY_PCT);
    gpio_set_drive_capability(IR_TRANSMITTER_GPIO, pulse_orig_drive);
    return ESP_OK;
}

/* ir_rxctl start|stop|status：手動控制 RMT RX 硬體（診斷用） */
static esp_err_t ir_rxctl_handler(int argc, char **argv)
{
    if (argc < 1) {
        ESP_LOGI(TAG, "Usage: matter esp ir_rxctl [start|stop|status]");
        return ESP_OK;
    }
    if (strcmp(argv[0], "start") == 0) {
        esp_err_t err = ir_rx_start();
        ESP_LOGI(TAG, "ir_rx_start() → %s", esp_err_to_name(err));
    } else if (strcmp(argv[0], "stop") == 0) {
        esp_err_t err = ir_rx_stop();
        ESP_LOGI(TAG, "ir_rx_stop() → %s", esp_err_to_name(err));
    } else if (strcmp(argv[0], "status") == 0) {
        ESP_LOGI(TAG, "rx_en_reg=%d filter_en=%d filter_thres=%u",
                 (int)RMT.chmconf[0].conf1.rx_en_chm,
                 (int)RMT.chmconf[0].conf1.rx_filter_en_chm,
                 (unsigned)RMT.chmconf[0].conf1.rx_filter_thres_chm);
        uint32_t cb_count, cb_partial, cb_last, cb_max_accum, cb_rearm;
        size_t last_cb_n;
        ir_rx_get_diag(&cb_count, &cb_partial, &cb_last, &cb_max_accum, &last_cb_n, &cb_rearm);
        ESP_LOGI(TAG, "diag: cb=%u partial=%u last=%u rearm=%u max_accum=%u last_n=%u",
                 (unsigned)cb_count, (unsigned)cb_partial, (unsigned)cb_last,
                 (unsigned)cb_rearm, (unsigned)cb_max_accum, (unsigned)last_cb_n);
    } else {
        ESP_LOGI(TAG, "Usage: matter esp ir_rxctl [start|stop|status]");
    }
    return ESP_OK;
}

namespace esp_matter {
namespace console {
static const command_t ir_commands[] = {
    {
        .name = "ir_test",
        .description = "測試 IR LED 發射。Usage: matter esp ir_test [count]",
        .handler = ir_test_handler,
    },
    {
        .name = "ir_carrier",
        .description = "載波測試：0=normal, 1=always_on, 2=no carrier. Usage: matter esp ir_carrier [0|1|2]",
        .handler = ir_carrier_handler,
    },
    {
        .name = "ir_reg",
        .description = "讀取 RMT TX register 診斷 carrier 設定",
        .handler = ir_reg_handler,
    },
    {
        .name = "ir_set_drive",
        .description = "設定 IR LED 驅動能力 0=weakest(5mA,loopback) ~ 3=strongest(40mA,AC). Usage: matter esp ir_set_drive [0|1|2|3]",
        .handler = ir_set_drive_handler,
    },
    {
        .name = "ir_pulse",
        .description = "發射單一 mark/space 測試 VS1838B 靈敏度。Usage: matter esp ir_pulse [mark_us] [space_us]",
        .handler = ir_pulse_handler,
    },
    {
        .name = "ir_loopback",
        .description = "IR loopback 測試：發射 HITACHI_AC1 → 接收 → 解碼 → 比對。Usage: matter esp ir_loopback [duty_pct 1-50]",
        .handler = ir_loopback_handler,
    },
    {
        .name = "ir_compare",
        .description = "按遙控器鍵，比較 encoder 與遙控器訊號是否一致",
        .handler = ir_compare_handler,
    },
    {
        .name = "ir_listen",
        .description = "持續監聽 IR 訊號並印出（預設 30 秒）。Usage: matter esp ir_listen [seconds]",
        .handler = ir_listen_handler,
    },
    {
        .name = "ir_roundtrip",
        .description = "純軟體 encode→decode 測試（不需 IR LED）",
        .handler = ir_roundtrip_handler,
    },
    {
        .name = "ir_send",
        .description = "發射 HITACHI_AC1 state 給冷氣（不接收）。Usage: matter esp ir_send [on|off] [temp 16-31] [cool|heat|fan|dry|auto] [repeat 2-15]",
        .handler = ir_send_handler,
    },
    {
        .name = "ir_replay",
        .description = "捕獲遙控器訊號並原樣重播（診斷 only-off-works 問題）。Usage: matter esp ir_replay [repeat 1-10]",
        .handler = ir_replay_handler,
    },
    {
        .name = "ir_dump",
        .description = "發射測試訊號並印出 RMT RX 收到的 raw symbols（診斷 RX 過濾問題）",
        .handler = ir_dump_handler,
    },
    {
        .name = "ir_rxctl",
        .description = "手動控制 RMT RX 硬體。Usage: matter esp ir_rxctl [start|stop|status]",
        .handler = ir_rxctl_handler,
    },
};
} // namespace console
} // namespace esp_matter

esp_err_t ir_console_init(void)
{
    return esp_matter::console::add_commands(esp_matter::console::ir_commands,
                                             sizeof(esp_matter::console::ir_commands) / sizeof(esp_matter::console::command_t));
}

/* RMT resolution：1MHz = 1us per tick，方便直接用 us */
#define RMT_RESOLUTION_HZ   1000000

/* RMT channel handles 已在檔案前方宣告告 */

/* 接收緩衝與 ring buffer（供 ir_get_last_received 取出） */
static rmt_symbol_word_t s_rx_symbols[IR_RAW_MAX_TIMINGS];
static RingbufHandle_t s_rx_ringbuf = NULL;

/* RX 接收設定（file-scope，callback 重 arm 時復用） */
static rmt_receive_config_t s_recv_cfg = {
    /* pulse filter：過濾短脈衝雜訊。
     * RMT RX filter 以 group clock (80MHz) 為單位，register 上限 255 (8-bit)。
     * max filter = 255 / 80MHz ≈ 3.19us → 設 3000ns (3us, 240 ticks) 保留 margin。
     * VS1838B 已硬體 demodulate 38kHz carrier，RMT 不需也不該做 carrier demod。
     * HITACHI_AC1 最小合法 pulse = BIT_MARK=400us，3us threshold 有充分 margin。 */
    .signal_range_min_ns = 3000,  /* 3us pulse filter (max ~3.19us @80MHz, 8-bit reg) */
    .signal_range_max_ns = 30000000,  /* 30ms；gap > 30ms 觸發 is_last */
    .flags = { .en_partial_rx = 1 },
};

/* 最近一次接收的 raw timing（us） */
static uint32_t s_last_timings[IR_RAW_MAX_TIMINGS];
static size_t s_last_count = 0;
static portMUX_TYPE s_last_mux = portMUX_INITIALIZER_UNLOCKED;

/* 預分配的展平緩衝（避免在 ISR critical section 內 malloc） */
static uint32_t s_flatten_buf[IR_RAW_MAX_TIMINGS * 2];

/* Callback 診斷計數器（用 atomic 操作，不需 lock） */
static volatile uint32_t s_cb_count = 0;       /* callback 總觸發次數 */
static volatile uint32_t s_cb_partial = 0;     /* is_last=0（partial）次數 */
static volatile uint32_t s_cb_last = 0;        /* is_last=1（結束）次數 */
static volatile uint32_t s_cb_rearm = 0;       /* callback 內重 arm RMT 次數 */
static volatile uint32_t s_cb_max_accum = 0;   /* 累積 symbols 最高水位 */
static volatile size_t s_last_cb_n = 0;        /* 最近一次 callback 的 n */

/* ========== Partial RX 累積緩衝 ==========
 * ESP32-C6 RMT 每通道 48 words，RX 設 96 words（2 通道）。
 * HITACHI_AC1 訊框 = 211 個 timing ≈ 106 個 symbol，超過 96，必須用 partial RX：
 * RMT 在 buffer 滿時觸發 callback（is_done=false），我們累積到組裝緩衝。
 */
static rmt_symbol_word_t s_rx_accum[IR_RAW_MAX_TIMINGS]; /* 累積用 symbol 緩衝 */
static size_t s_rx_accum_count = 0;
static portMUX_TYPE s_rx_accum_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t s_last_cb_time_us = 0;  /* 最後一次 callback 時間（超時 flush 用） */
static volatile bool s_rx_exclusive = false;   /* true 時 app_driver 不可讀取 RX 資料 */
static volatile bool s_rx_hw_enabled = false;  /* RMT RX 硬體是否 enable（idempotent stop/start） */
#define IR_RX_GAP_THRESHOLD_US  20000  /* >20ms 的 space 視為訊框結束（gap） */
#define IR_RX_FLUSH_TIMEOUT_US 150000  /* 150ms 無新 callback 視為訊框完整 */
#define IR_RX_EXPECTED_SYMBOLS 106      /* HITACHI_AC1 期望 symbol 數（211 timings ≈ 106 symbols） */

/* ========== TX：預轉換 symbol 陣列 + rmt_copy_encoder ==========
 * 原本用自定義 stateless encoder，但 212 個 symbol 超過 RMT buffer 時，
 * encode 函數每次從頭開始，後面的 symbol 被截斷。
 * 改用預轉換：把 timings 先轉成 rmt_symbol_word_t 陣列，
 * 直接交給 rmt_copy_encoder，它本身是 stateful，正確處理 streaming 分段傳送。
 *
 * 軟體載波調變後的 symbol 數量遠大於原始 timing 數量。
 * 每個 mark 需要多個 carrier cycle symbols，每個 space 只需 1 個 symbol。
 * 38kHz @ 1MHz：carrier period=26us，50% duty → high=13us, low=13us
 * 3400us mark ≈ 131 cycles, 400us mark ≈ 16 cycles, 總計約 1918 symbols
 * （僅在硬體 carrier 失效時作為 fallback，正常使用硬體 carrier 只需 ~212 symbols）
 * 硬體 carrier data-phase-only 模式：每個 symbol = 1 mark+space pair
 * （duration0=mark level0=1 carrier on, duration1=space level1=0 carrier off）。
 * HITACHI_AC1 = 212 timings = 106 pairs，GAP=100000us 拆分 +2 → ~108 symbols。
 * 設 256 保留充足餘裕（也涵蓋 fallback 模式）。 */
#define IR_TX_SYMBOLS_MAX  256
static rmt_symbol_word_t s_tx_symbols[IR_TX_SYMBOLS_MAX]; /* 預轉換緩衝 */

/* ========== RX callback（支援 partial RX 累積） ==========
 * 注意：此 callback 在 ISR context 執行，嚴禁 malloc / ESP_LOGI 等 blocking 操作。
 * 展平用預分配的 s_flatten_buf，log 只用 ESP_LOGD（debug level，預設不輸出）。
 */
static bool ir_rx_done_cb(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata,
                          void *user_ctx)
{
    size_t n = edata->num_symbols;
    bool is_last = edata->flags.is_last;

    /* 診斷計數（ISR-safe，volatile uint32_t 写入是原子的 on ESP32） */
    s_cb_count++;
    if (is_last) s_cb_last++;
    else s_cb_partial++;
    s_last_cb_n = n;
    s_last_cb_time_us = esp_timer_get_time();

    if (n == 0) {
        return true;
    }

    /* Debug capture：task 要求捕獲下一批 raw symbols 供 ir_dump 印出 */
    if (s_debug_capture && n <= 256) {
        for (size_t i = 0; i < n; i++) {
            s_debug_symbols[i] = edata->received_symbols[i];
        }
        s_debug_symbol_count = n;
        s_debug_capture = false;
    }

    /* --- 臨界區：只做累積 + 展平 + 複製，不做 malloc / log ---
     * 多段訊框：is_last 不再視為訊框結束（因為 RMT 的 signal_range_max_ns
     * 上限 ~32ms，無法跨越段間 gap）。只有資料內的長 gap 或累積到期望 symbol
     * 數才標記完成。is_last 後 RMT 自動重啟接收，下一段繼續累積。 */
    portENTER_CRITICAL_ISR(&s_rx_accum_mux);

    bool frame_complete = false;
    for (size_t i = 0; i < n; i++) {
        if (s_rx_accum_count < IR_RAW_MAX_TIMINGS) {
            s_rx_accum[s_rx_accum_count++] = edata->received_symbols[i];
        } else {
            s_rx_accum_count = 0;
            s_rx_accum[s_rx_accum_count++] = edata->received_symbols[i];
        }
        if (edata->received_symbols[i].duration1 >= IR_RX_GAP_THRESHOLD_US) {
            frame_complete = true;
        }
    }
    /* 累積到期望 symbol 數也標記完成 */
    if (s_rx_accum_count >= IR_RX_EXPECTED_SYMBOLS) {
        frame_complete = true;
    }
    /* 注意：is_last 不再標記 frame_complete（多段訊框處理） */

        /* 更新最高水位 */
    if (s_rx_accum_count > s_cb_max_accum) {
        s_cb_max_accum = s_rx_accum_count;
    }

    size_t flatten_count = 0;
    if (frame_complete && s_rx_accum_count > 0) {
        /* 用預分配緩衝展平 symbols → us timing 陣列 */
        for (size_t i = 0; i < s_rx_accum_count && flatten_count < (IR_RAW_MAX_TIMINGS * 2); i++) {
            if (s_rx_accum[i].duration0) {
                s_flatten_buf[flatten_count++] = s_rx_accum[i].duration0;
            }
            if (s_rx_accum[i].duration1) {
                s_flatten_buf[flatten_count++] = s_rx_accum[i].duration1;
            }
        }
        s_rx_accum_count = 0;
    }

    /* 複製到 last_timings（供 ir_get_last_received） */
    if (flatten_count > 0 && flatten_count <= IR_RAW_MAX_TIMINGS) {
        portENTER_CRITICAL_ISR(&s_last_mux);
        memcpy(s_last_timings, s_flatten_buf, flatten_count * sizeof(uint32_t));
        s_last_count = flatten_count;
        portEXIT_CRITICAL_ISR(&s_last_mux);
    }

    portEXIT_CRITICAL_ISR(&s_rx_accum_mux);

    /* --- 臨界區外：ring buffer 寫入（非阻塞） --- */
    if (flatten_count > 0 && s_rx_ringbuf) {
        xRingbufferSend(s_rx_ringbuf, s_flatten_buf, flatten_count * sizeof(uint32_t), 0);
    }

    /* 重 arm RMT：is_last 後 RMT 自動停機（fsm=ENABLE），必須重新呼叫
     * rmt_receive() 才能接收下一段。rmt_receive() 是 ISR-safe（用
     * ESP_RETURN_ON_FALSE_ISR + portENTER_CRITICAL_SAFE）。復用同一個
     * s_rx_symbols 緩衝與 s_recv_cfg 設定。這是擷取多段訊框的關鍵。 */
    if (is_last) {
        rmt_receive(s_rx_channel, s_rx_symbols, sizeof(s_rx_symbols), &s_recv_cfg);
        s_cb_rearm++;
    }

    return true;
}

/* ========== 公開 API ==========
 * ir_transmit_raw：硬體載波版本（mark+space pair symbol）
 *
 * ESP32-C6 RMT 支援 TX carrier data-phase-only 模式（carrier_eff_en_chn=1）：
 * carrier 只在 symbol 的 level=1（mark）相位輸出，level=0（space）和 idle 自動關閉。
 *
 * Pair symbol 編碼：每個 RMT symbol 的 duration0=mark（level0=1, carrier on），
 * duration1=space（level1=0, carrier off）。如此 212 timings（106 mark+space pairs）
 * 只需 ~108 symbols（GAP 100000us 需拆分 +2），vs 舊版 212 symbols。
 * Ping-pong half=24 symbols 的時間預算從 ~24ms 提升到 ~47ms，
 * 大幅降低 ESP32-C6 single-core ISR 來不及 refill 的風險。
 *
 * 物理輸出與舊版（level0=level1 同 level）完全相同：data-phase-only 模式下
 * carrier 依每個 phase 的 level 決定開關，mark=carrier on, space=carrier off。
 *
 * Carrier 參數由 rmt_apply_carrier() 設定：
 *   TX carrier 以 group clock (80MHz) 為基準計算（rmt_tx.c:887）：
 *     total_ticks = 80M/38k = 2105, high=1052, low=1053
 *     real_freq = 80M/2105 = 38004 Hz（誤差 0.01%）
 *   注意：RX demodulation 以 channel clock (1MHz) 為基準（rmt_rx.c:465），
 *   兩者 base clock 不同但算出的頻率結果一致
 *
 * 注意：RMT symbol duration 最多 32767us（15-bit），超過需拆分。
 *       duration1=0 會被 RMT 視為 stop marker，需確保 >= 1。
 */
esp_err_t ir_transmit_raw(const uint32_t *timings, size_t count)
{
    if (!s_tx_channel || !timings || count == 0) return ESP_ERR_INVALID_STATE;
    if (count > IR_RAW_MAX_TIMINGS) return ESP_ERR_INVALID_SIZE;

    /* Pair symbol: duration0=mark (level0=1, carrier on),
     *              duration1=space (level1=0, carrier off).
     * timings 交替 mark/space，even index=mark, odd index=space. */
    size_t sym_idx = 0;
    for (size_t i = 0; i < count && sym_idx < IR_TX_SYMBOLS_MAX; i += 2) {
        uint32_t mark = timings[i];
        uint32_t space = (i + 1 < count) ? timings[i + 1] : 0;

        /* Phase 1: emit mark. 若 mark <= 32767，直接 pair 進 d0，space 進 d1。 */
        while (mark > 0 && sym_idx < IR_TX_SYMBOLS_MAX) {
            uint32_t d0 = (mark > 32767) ? 32767 : mark;
            mark -= d0;

            if (mark == 0 && space > 0) {
                /* mark 用完 — pair mark tail(d0) 與 space start(d1) */
                uint32_t d1 = (space > 32767) ? 32767 : space;
                space -= d1;
                s_tx_symbols[sym_idx].duration0 = (uint16_t)d0;
                s_tx_symbols[sym_idx].level0 = 1;
                s_tx_symbols[sym_idx].duration1 = (uint16_t)d1;
                s_tx_symbols[sym_idx].level1 = 0;
            } else if (mark == 0 && space == 0) {
                /* trailing mark（odd count, 無後續 space） */
                uint32_t d1 = 1;
                if (d0 > 1) d0 -= 1;
                s_tx_symbols[sym_idx].duration0 = (uint16_t)d0;
                s_tx_symbols[sym_idx].level0 = 1;
                s_tx_symbols[sym_idx].duration1 = (uint16_t)d1;
                s_tx_symbols[sym_idx].level1 = 1;
            } else {
                /* mark > 32767 需拆分 — 兩 phase 都 mark（carrier on） */
                uint32_t d1 = (mark > 32767) ? 32767 : mark;
                mark -= d1;
                s_tx_symbols[sym_idx].duration0 = (uint16_t)d0;
                s_tx_symbols[sym_idx].level0 = 1;
                s_tx_symbols[sym_idx].duration1 = (uint16_t)d1;
                s_tx_symbols[sym_idx].level1 = 1;
            }
            sym_idx++;
        }

        /* Phase 2: emit 殘餘 space（mark 已用完但 space 溢出 d1） */
        while (space > 0 && sym_idx < IR_TX_SYMBOLS_MAX) {
            uint32_t d0 = (space > 32767) ? 32767 : space;
            space -= d0;
            uint32_t d1;
            if (space > 0) {
                d1 = (space > 32767) ? 32767 : space;
                space -= d1;
            } else {
                d1 = 1;
                if (d0 > 1) d0 -= 1;
            }
            s_tx_symbols[sym_idx].duration0 = (uint16_t)d0;
            s_tx_symbols[sym_idx].level0 = 0;
            s_tx_symbols[sym_idx].duration1 = (uint16_t)d1;
            s_tx_symbols[sym_idx].level1 = 0;
            sym_idx++;
        }
    }
    size_t total_symbols = sym_idx;

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags = { .eot_level = 0, .queue_nonblocking = 0 },
    };
    esp_err_t err = rmt_transmit(s_tx_channel, s_copy_encoder, s_tx_symbols,
                                 total_symbols * sizeof(rmt_symbol_word_t), &tx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_transmit failed: %s (symbols=%u)", esp_err_to_name(err), (unsigned)total_symbols);
        return err;
    }
    err = rmt_tx_wait_all_done(s_tx_channel, 2000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_tx_wait_all_done failed: %s (symbols=%u)", esp_err_to_name(err), (unsigned)total_symbols);
        return err;
    }
    ESP_LOGI(TAG, "IR TX done: %u timings → %u symbols (hardware carrier)",
             (unsigned)count, (unsigned)total_symbols);
    return ESP_OK;
}

esp_err_t ir_get_last_received(uint32_t *out_buf, size_t buf_size, size_t *out_count)
{
    if (!out_buf || !out_count) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_last_mux);
    if (s_last_count == 0) {
        portEXIT_CRITICAL(&s_last_mux);
        return ESP_ERR_NOT_FOUND;
    }
    size_t copy = (s_last_count < buf_size) ? s_last_count : buf_size;
    memcpy(out_buf, s_last_timings, copy * sizeof(uint32_t));
    *out_count = copy;
    /* 清除 count，避免重複讀取同一筆數據 */
    s_last_count = 0;
    portEXIT_CRITICAL(&s_last_mux);
    return ESP_OK;
}

/* ========== Callback 診斷（供外部 log partial RX 行為） ========== */
void ir_rx_get_diag(uint32_t *cb_count, uint32_t *cb_partial, uint32_t *cb_last,
                    uint32_t *cb_max_accum, size_t *last_cb_n, uint32_t *cb_rearm)
{
    if (cb_count) *cb_count = s_cb_count;
    if (cb_partial) *cb_partial = s_cb_partial;
    if (cb_last) *cb_last = s_cb_last;
    if (cb_max_accum) *cb_max_accum = s_cb_max_accum;
    if (last_cb_n) *last_cb_n = s_last_cb_n;
    if (cb_rearm) *cb_rearm = s_cb_rearm;
}

/* ========== 超時 flush（由 RX task 呼叫） ==========
 * 多段訊框：is_last 後 RMT 重啟接收，但若遙控器只發一段就停了（單段訊框），
 * 沒有後續 callback 觸發 frame_complete。此函數檢查超時，若距最後一次
 * callback > IR_RX_FLUSH_TIMEOUT_US 且有累積資料，展平並存到 s_last_timings。
 * 回傳 true 表示有 flush 出新訊框。
 */
bool ir_rx_flush_stale(void)
{
    if (s_last_cb_time_us == 0) return false;

    int64_t now = esp_timer_get_time();
    if ((now - s_last_cb_time_us) < IR_RX_FLUSH_TIMEOUT_US) {
        return false;  /* 尚未超時 */
    }

    size_t flatten_count = 0;
    portENTER_CRITICAL(&s_rx_accum_mux);
    if (s_rx_accum_count == 0) {
        portEXIT_CRITICAL(&s_rx_accum_mux);
        s_last_cb_time_us = 0;
        return false;
    }

    /* 展平累積 symbols → us timing 陣列 */
    for (size_t i = 0; i < s_rx_accum_count && flatten_count < (IR_RAW_MAX_TIMINGS * 2); i++) {
        if (s_rx_accum[i].duration0) {
            s_flatten_buf[flatten_count++] = s_rx_accum[i].duration0;
        }
        if (s_rx_accum[i].duration1) {
            s_flatten_buf[flatten_count++] = s_rx_accum[i].duration1;
        }
    }
    s_rx_accum_count = 0;
    portEXIT_CRITICAL(&s_rx_accum_mux);

    /* 存到 last_timings */
    if (flatten_count > 0 && flatten_count <= IR_RAW_MAX_TIMINGS) {
        portENTER_CRITICAL(&s_last_mux);
        memcpy(s_last_timings, s_flatten_buf, flatten_count * sizeof(uint32_t));
        s_last_count = flatten_count;
        portEXIT_CRITICAL(&s_last_mux);
    }

    s_last_cb_time_us = 0;
    return true;
}

void ir_rx_clear(void)
{
    portENTER_CRITICAL(&s_rx_accum_mux);
    s_rx_accum_count = 0;
    portEXIT_CRITICAL(&s_rx_accum_mux);

    portENTER_CRITICAL(&s_last_mux);
    s_last_count = 0;
    portEXIT_CRITICAL(&s_last_mux);

    s_last_cb_time_us = 0;
    s_cb_count = 0;
    s_cb_partial = 0;
    s_cb_last = 0;
    s_cb_rearm = 0;
    s_cb_max_accum = 0;
    s_last_cb_n = 0;

    /* 清空 ring buffer */
    if (s_rx_ringbuf) {
        size_t item_size;
        void *item;
        while ((item = xRingbufferReceive(s_rx_ringbuf, &item_size, 0)) != NULL) {
            vRingbufferReturnItem(s_rx_ringbuf, item);
        }
    }
}

bool ir_rx_is_exclusive(void)
{
    return s_rx_exclusive;
}

void ir_rx_set_exclusive(bool exclusive)
{
    s_rx_exclusive = exclusive;
}

/* 停止 RMT RX 硬體：避免 TX 時接收自己的訊號。
 * Idempotent：已停止時直接返回 OK。
 * 注意：即使 rmt_disable 失敗（可能已 disabled），仍強制清除 flag，
 *       避免後續 ir_rx_start 被誤認為已啟動而跳過重啟。 */
esp_err_t ir_rx_stop(void)
{
    if (!s_rx_channel) return ESP_OK;
    if (!s_rx_hw_enabled) return ESP_OK;
    esp_err_t err = rmt_disable(s_rx_channel);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "rmt_disable rx: %s (may already be disabled)", esp_err_to_name(err));
    }
    s_rx_hw_enabled = false;  /* 無論 rmt_disable 成功與否都清除 flag */
    return ESP_OK;
}

/* 重新啟動 RMT RX 硬體：force disable → enable → 清除緩衝 → re-arm 接收。
 * 非冪等：每次呼叫都強制重新啟動，確保 flag 與硬體狀態一致。
 * 注意：rmt_receive 回傳值必須檢查，否則 flag 會與硬體狀態不同步。 */
esp_err_t ir_rx_start(void)
{
    if (!s_rx_channel) return ESP_OK;
    /* 先 disable 確保 FSM 回到 INIT（safe even if already disabled） */
    rmt_disable(s_rx_channel);
    esp_err_t err = rmt_enable(s_rx_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable rx failed: %s", esp_err_to_name(err));
        s_rx_hw_enabled = false;
        return err;
    }
    ir_rx_clear();
    err = rmt_receive(s_rx_channel, s_rx_symbols, sizeof(s_rx_symbols), &s_recv_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_receive failed: %s", esp_err_to_name(err));
        s_rx_hw_enabled = false;
        return err;
    }
    s_rx_hw_enabled = true;
    return ESP_OK;
}

esp_err_t ir_driver_init(void)
{
    esp_err_t err;

    /* ===== TX channel ===== */
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = IR_TRANSMITTER_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        /* ESP32-C6 每通道 48 words。設 48 = 只佔 1 個 TX 通道。
         * trans_queue_depth=4 讓 rmt_copy_encoder 處理超過 48 symbols 的長訊框
         * （HITACHI_AC1 ≈ 212 symbols），RMT 硬體會自動 ping-pong 重載。 */
        .mem_block_symbols = 48,
        .trans_queue_depth = 4,
        .intr_priority = 0,
        .flags = {
            .invert_out = 0,
            .with_dma = 0,
            .io_loop_back = 0,
            .io_od_mode = 0,
            .allow_pd = 0,
            .init_level = 0,
        },
    };
    err = rmt_new_tx_channel(&tx_cfg, &s_tx_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 啟用硬體 carrier：38kHz data-phase-only 模式（比照 IRremoteESP8266 rmt_on_esp32 branch）
     * carrier_en=true, carrier_level=HIGH, duty=50%, freq=38kHz
     * ESP32-C6 carrier 以 group clock (80MHz) 為基準計算：
     *   total_ticks = 80M/38k = 2105, high=1052, low=1053
     *   real_freq = 80M/2105 = 38004 Hz（誤差 0.01%）
     * carrier_eff_en_chn=1（預設）：carrier 只在 mark（level=1）輸出，
     * space（level=0）和 idle 自動關閉。 */
    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = IR_CARRIER_FREQ_HZ,
        .duty_cycle = (IR_CARRIER_DUTY_PCT / 100.0f),
        .flags = {
            .polarity_active_low = 0,
            .always_on = 0,
        },
    };
    err = rmt_apply_carrier(s_tx_channel, &carrier_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_apply_carrier failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "IR hardware carrier enabled: %uHz @ %u%% duty (data-phase-only)",
             (unsigned)IR_CARRIER_FREQ_HZ, (unsigned)IR_CARRIER_DUTY_PCT);

    /* 建立 copy encoder（直接傳送 rmt_symbol_word_t 陣列，stateful 處理 streaming） */
    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &s_copy_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_copy_encoder failed: %s", esp_err_to_name(err));
        return err;
    }

    err = rmt_enable(s_tx_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable tx failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 設定 IR LED GPIO 為最強驅動能力（GPIO_DRIVE_CAP_3 ≈ 40mA）
     * 配合 200Ω 電阻：峰值電流 ≈ (3.3V - 1.4V) / 200Ω ≈ 9.5mA
     * 50% carrier duty cycle：mark 平均電流 ≈ 4.75mA
     * 注意：loopback 時此驅動力會導致 VS1838B AGC 飽和（只收到 header mark），
     * 但真實冷氣接收器在 1-3m 距離不受影響。
     * 如需 loopback 測試，可用 `matter esp ir_set_drive 0` 降低驅動。 */
    gpio_set_drive_capability(IR_TRANSMITTER_GPIO, GPIO_DRIVE_CAP_3);
    gpio_drive_cap_t drv_cap;
    gpio_get_drive_capability(IR_TRANSMITTER_GPIO, &drv_cap);
    ESP_LOGI(TAG, "IR TX GPIO%d drive capability set to %d (0=weakest, 3=strongest)", IR_TRANSMITTER_GPIO, (int)drv_cap);

    /* ===== RX channel ===== */
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = IR_RECEIVER_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        /* ESP32-C6 每通道 48 words，2 個 RX 通道共 96 words。
         * mem_block_symbols 必須 <= 96，否則需 3 個連續通道（RX 只有 2 個，會失敗）。
         * 設 96 = 剛好填滿 2 個 RX 通道（ping-pong 各 48 words）。 */
        .mem_block_symbols = 96,
        .intr_priority = 0,
        .flags = {
            /* VS1838B 輸出 active-low：收到載波時拉低。
             * RMT level=1 視為 mark，所以 invert_in=true 讓載波期間 level=1 */
            .invert_in = 1,
            .with_dma = 0,
            .io_loop_back = 0,
            .allow_pd = 0,
        },
    };
    err = rmt_new_rx_channel(&rx_cfg, &s_rx_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_rx_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    rmt_rx_event_callbacks_t rx_cbs = {
        .on_recv_done = ir_rx_done_cb,
    };
    err = rmt_rx_register_event_callbacks(s_rx_channel, &rx_cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_rx_register_event_callbacks failed: %s", esp_err_to_name(err));
        return err;
    }

    /* ring buffer 存接收到的 timing（供 Phase 2 解碼） */
    s_rx_ringbuf = xRingbufferCreate(IR_RAW_MAX_TIMINGS * sizeof(uint32_t) * 2, RINGBUF_TYPE_NOSPLIT);
    if (!s_rx_ringbuf) {
        ESP_LOGE(TAG, "xRingbufferCreate failed");
        return ESP_ERR_NO_MEM;
    }

    err = rmt_enable(s_rx_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable rx failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 啟動第一次接收：用 partial RX 累積長訊框（HITACHI_AC1 ≈ 106 symbols）。
     * signal_range_max_ns 設 30ms（RMT 限制 < 32767000 ns）：
     *   - 正常 space 最長 HDR_SPACE=3400us < 30ms，不會誤判結束
     *   - 訊框結束的 gap=10s > 30ms，會觸發 is_last=1 結束接收
     * 訊框結束由 callback 偵測 is_last 決定。
     * is_last 後 callback 會自動重 arm RMT 接收下一段。 */
    rmt_receive(s_rx_channel, s_rx_symbols, sizeof(s_rx_symbols), &s_recv_cfg);
    s_rx_hw_enabled = true;

    ESP_LOGI(TAG, "IR driver ready: TX=GPIO%d (38kHz), RX=GPIO%d (inverted)",
             IR_TRANSMITTER_GPIO, IR_RECEIVER_GPIO);

    return ESP_OK;
}
