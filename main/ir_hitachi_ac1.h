/*
 * Hitachi AC MITM - ir_hitachi_ac1.h
 *
 * HITACHI_AC1 紅外線協議的解碼/編碼（移植自 IRremoteESP8266 函式庫）
 *
 * 來源：
 *   https://github.com/crankyoldgit/IRremoteESP8266
 *   src/ir_Hitachi.h  - HitachiAc1Protocol union 結構與常數
 *   src/ir_Hitachi.cpp - sendHitachiAC1 / decodeHitachiAC / IRHitachiAc1 類別
 *
 * HITACHI_AC1 規格：
 *   - 13 bytes / 104 bits
 *   - Header: mark 3400us, space 3400us
 *   - Bit:    mark 400us
 *   - One:    space 1250us
 *   - Zero:   space 500us
 *   - Gap:    ~100ms (kHitachiAcMinGap = kDefaultMessageGap = 100000us)
 *   - Bit order: MSB first
 *   - 載波：38kHz
 *
 * State 結構（13 bytes）：
 *   Byte 0-4: 固定頭 (0xB2, 0xAE, 0x4D, 0x91, 0xF0)
 *   Byte 5: Mode(高4) + Fan(低4)
 *   Byte 6: Temp(bits 2-6, 5 bits reversed)
 *   Byte 7-8: OffTimer
 *   Byte 9-10: OnTimer
 *   Byte 11: SwingToggle(bit0) + Sleep(bits1-3) + PowerToggle(bit4) + Power(bit5) + SwingV(bit6) + SwingH(bit7)
 *   Byte 12: Checksum
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 協議常數 ========== */

#define HITACHI_AC1_STATE_LEN    13    /* kHitachiAc1StateLength */
#define HITACHI_AC1_BITS         104   /* kHitachiAc1Bits */
#define HITACHI_AC1_FREQ_HZ      38000

/* Timing (us) - 來自 IRremoteESP8266 參考實作 */
#define HITACHI_AC1_HDR_MARK     3400
#define HITACHI_AC1_HDR_SPACE    3400
#define HITACHI_AC1_BIT_MARK     400
#define HITACHI_AC1_ONE_SPACE    1250
#define HITACHI_AC1_ZERO_SPACE   500
/* kHitachiAcMinGap = kDefaultMessageGap = 100000us。
 * 但 RMT symbol duration 最多 32767us（15-bit），
 * 實際發射時由 ir_transmit_raw() 拆成多個 symbol。
 * AC 接收器靠 timeout 偵測訊框結束，gap 只需 > ~10ms 即可。 */
#define HITACHI_AC1_GAP          100000

/* Model 值（Byte 11 bit 6-7） */
#define HITACHI_AC1_MODEL_A      0b10
#define HITACHI_AC1_MODEL_B      0b01

/* Mode 值（Byte 5 高 4 bits）— 來自 IRremoteESP8266 */
#define HITACHI_AC1_MODE_DRY     2   /* 0b0010 */
#define HITACHI_AC1_MODE_FAN     4   /* 0b0100 */
#define HITACHI_AC1_MODE_COOL    6   /* 0b0110 */
#define HITACHI_AC1_MODE_HEAT    9   /* 0b1001 */
#define HITACHI_AC1_MODE_AUTO    14  /* 0b1110 */

/* Fan 值（Byte 10 高 4 bits） */
#define HITACHI_AC1_FAN_AUTO     1   /* 0b0001 */
#define HITACHI_AC1_FAN_HIGH     2   /* 0b0010 */
#define HITACHI_AC1_FAN_MED      4   /* 0b0100 */
#define HITACHI_AC1_FAN_LOW      8   /* 0b1000 */

/* 溫度範圍 */
#define HITACHI_AC1_MIN_TEMP     16
#define HITACHI_AC1_MAX_TEMP     32
#define HITACHI_AC1_TEMP_AUTO    25
#define HITACHI_AC1_TEMP_DELTA   7

/* ========== State 結構 ========== */

typedef struct {
    uint8_t raw[HITACHI_AC1_STATE_LEN];
} hitachi_ac1_state_t;

/* ========== 公開 API ========== */

/**
 * 重置 state 為已知良好狀態
 */
void hitachi_ac1_state_reset(hitachi_ac1_state_t *state);

/* --- 欄位存取 --- */
void     hitachi_ac1_set_power(hitachi_ac1_state_t *s, bool on);
void     hitachi_ac1_sync_power(hitachi_ac1_state_t *s, bool on);  /* 同步 Power bit 但不設 PowerToggle（GPIO 同步用） */
bool     hitachi_ac1_get_power(const hitachi_ac1_state_t *s);
void     hitachi_ac1_set_power_toggle(hitachi_ac1_state_t *s, bool on);
bool     hitachi_ac1_get_power_toggle(const hitachi_ac1_state_t *s);
void     hitachi_ac1_set_temp(hitachi_ac1_state_t *s, uint8_t celsius);
uint8_t  hitachi_ac1_get_temp(const hitachi_ac1_state_t *s);
void     hitachi_ac1_set_mode(hitachi_ac1_state_t *s, uint8_t mode);
uint8_t  hitachi_ac1_get_mode(const hitachi_ac1_state_t *s);
void     hitachi_ac1_set_fan(hitachi_ac1_state_t *s, uint8_t speed);
uint8_t  hitachi_ac1_get_fan(const hitachi_ac1_state_t *s);
void     hitachi_ac1_set_model(hitachi_ac1_state_t *s, uint8_t model);
uint8_t  hitachi_ac1_get_model(const hitachi_ac1_state_t *s);

/* --- 編碼 / 解碼 --- */

/**
 * 把 state 編碼成 us timing 陣列，供 ir_transmit_raw() 發射。
 * HITACHI_AC1 = header + 104 bits + footer mark + gap
 *
 * @param state    輸入 state
 * @param timings  輸出 us timing 陣列
 * @param max_count timings 陣列最大容量
 * @param out_count  實際寫入的元素數
 * @return ESP_OK 或錯誤碼
 */
esp_err_t hitachi_ac1_encode(const hitachi_ac1_state_t *state,
                             uint32_t *timings, size_t max_count, size_t *out_count);

/**
 * 從 us timing 陣列解碼出 state。
 * 會驗證 header timing、bit 數與 checksum。
 *
 * @param timings  輸入 us timing 陣列
 * @param count    timings 元素數
 * @param state    輸出 state
 * @return ESP_OK 或 ESP_ERR_INVALID_VERSION / ESP_ERR_CRC
 */
esp_err_t hitachi_ac1_decode(const uint32_t *timings, size_t count,
                             hitachi_ac1_state_t *state);

/**
 * 計算 checksum（移植自 IRHitachiAc1::calcChecksum）
 */
uint8_t hitachi_ac1_calc_checksum(const uint8_t *state, size_t length);

/**
 * 驗證 checksum
 */
bool hitachi_ac1_check_checksum(const uint8_t *state, size_t length);

/**
 * 傾印 state 內容到 log
 */
void hitachi_ac1_state_log(const hitachi_ac1_state_t *state);

#ifdef __cplusplus
}
#endif
