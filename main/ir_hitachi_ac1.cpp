/*
 * Hitachi AC MITM - ir_hitachi_ac1.cpp
 *
 * HITACHI_AC1 協議解碼/編碼實作（移植自 IRremoteESP8266）
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "ir_hitachi_ac1.h"

static const char *TAG = "ir_hitachi_ac1";

/* ========== 內部輔助：reverse bits（移植自 IRutils） ========== */
static uint8_t reverse_bits_8(uint8_t in)
{
    uint8_t out = 0;
    for (int i = 0; i < 8; i++) {
        out = (out << 1) | (in & 1);
        in >>= 1;
    }
    return out;
}

static uint8_t reverse_nibble(uint8_t in)
{
    return reverse_bits_8(in) >> 4;
}

/* 反轉低 5 bits（用於溫度欄位，移植自 IRremoteESP8266 reverseBits(x, 5)） */
static uint8_t reverse_5bits(uint8_t in)
{
    return ((in & 0x01) << 4) |
           ((in & 0x02) << 2) |
           (in & 0x04) |
           ((in & 0x08) >> 2) |
           ((in & 0x10) >> 4);
}

/* ========== Checksum（移植自 IRHitachiAc1::calcChecksum） ==========
 * Byte 5 到 length-2 的每個 byte，取低/高 nibble 分別 reverse 後相加，
 * 最後總和 reverse 即為 checksum。
 */
uint8_t hitachi_ac1_calc_checksum(const uint8_t *state, size_t length)
{
    if (length < 2) return 0;

    uint8_t sum = 0;
    /* kHitachiAc1ChecksumStartByte = 5 */
    for (size_t i = 5; i < length - 1; i++) {
        sum += reverse_nibble(state[i] & 0x0F);
        sum += reverse_nibble(state[i] >> 4);
    }
    return reverse_bits_8(sum);
}

bool hitachi_ac1_check_checksum(const uint8_t *state, size_t length)
{
    if (length < 2) return true;
    return state[length - 1] == hitachi_ac1_calc_checksum(state, length);
}

/* ========== stateReset ========== */
void hitachi_ac1_state_reset(hitachi_ac1_state_t *state)
{
    if (!state) return;
    memset(state->raw, 0, HITACHI_AC1_STATE_LEN);

    /* 固定頭 */
    state->raw[0] = 0xB2;
    state->raw[1] = 0xAE;
    state->raw[2] = 0x4D;
    state->raw[3] = 0x91;
    state->raw[4] = 0xF0;

    /* 預設：Model A, Power ON, Auto mode, 25°C (kHitachiAc1TempAuto), Auto Fan
     * 與官方 stateReset 一致：raw[5]=0xE1 (Auto|Auto), raw[6]=0xA4 (25°C)
     * 注意：set_temp(23) 在 AUTO mode 下是 no-op（官方行為），故實際 temp=25°C */
    state->raw[5] = 0xE1;  /* Byte5: Mode=Auto(0xE) | Fan=Auto(0x1) */
    state->raw[6] = 0xA4;  /* Byte6: Temp=25°C (kHitachiAc1TempAuto) */
    state->raw[7] = 0x00;  /* Byte7-8: OffTimer */
    state->raw[8] = 0x00;
    state->raw[9] = 0x00;  /* Byte9-10: OnTimer */
    state->raw[10] = 0x00;
    state->raw[11] = 0x00;  /* Byte11: Power=0, PowerToggle=0, SwingToggle=0, SwingV=0
                             * 實體遙控器永遠送 Power=0（只靠 PowerToggle 切換電源），
                             * MITM 模擬遙控器：Pwr bit 始終 0，實際電源狀態由 GPIO 判讀。 */

    hitachi_ac1_set_model(state, HITACHI_AC1_MODEL_A);
    hitachi_ac1_set_temp(state, 23);  /* AUTO mode 下 no-op，保留 raw[6]=0xA4 (25°C) */
    /* 不呼叫 set_power() — set_power(true) 在 Power 已是 1 時不會設 toggle，
     * 但若未來改為 always-toggle 則會。state_reset 是初始化預設狀態，
     * 不應帶 toggle，直接寫 raw 即可 */

    /* 計算 checksum */
    state->raw[12] = hitachi_ac1_calc_checksum(state->raw, HITACHI_AC1_STATE_LEN);
}

/* ========== 欄位存取 ========== */

void hitachi_ac1_set_power(hitachi_ac1_state_t *s, bool on)
{
    if (!s) return;
    /* 官方邏輯：只在 Power 狀態「改變」時才設 PowerToggle=true（edge-triggered）
     * 若 Power 已是期望值（無變化），不設 toggle → AC 收到「狀態更新」而非「電源命令」
     * 參考 IRremoteESP8266 ir_Hitachi.cpp:539-543 setPower()
     *
     * 注意：MITM app_driver 層會在收到使用者開/關機命令時強制設 PowerToggle=true，
     * 因為遙控器開關機是同一個按鈕，不論 state 是否改變都應送 toggle。 */
    bool current_power = (s->raw[11] & 0x20) != 0;
    if (on != current_power) {
        s->raw[11] |= 0x10;  /* 設 PowerToggle bit (Byte 11 bit 4) */
    }
    /* Power 位在 Byte 11 bit 5 */
    if (on) s->raw[11] |= 0x20;
    else    s->raw[11] &= ~0x20;
    s->raw[12] = hitachi_ac1_calc_checksum(s->raw, HITACHI_AC1_STATE_LEN);
}

/* 同步 Power 狀態但不設 PowerToggle（供 GPIO 偵測 AC 實際電源變化時使用）
 * 與 set_power 差異：只更新 Power bit，不觸發 toggle → 下次 IR send 不會誤切電源 */
void hitachi_ac1_sync_power(hitachi_ac1_state_t *s, bool on)
{
    if (!s) return;
    if (on) s->raw[11] |= 0x20;
    else    s->raw[11] &= ~0x20;
    s->raw[12] = hitachi_ac1_calc_checksum(s->raw, HITACHI_AC1_STATE_LEN);
}

bool hitachi_ac1_get_power(const hitachi_ac1_state_t *s)
{
    if (!s) return false;
    return (s->raw[11] & 0x20) != 0;  /* Byte 11, bit 5 = Power */
}

void hitachi_ac1_set_power_toggle(hitachi_ac1_state_t *s, bool on)
{
    if (!s) return;
    /* PowerToggle 位在 Byte 11 bit 4 */
    if (on) s->raw[11] |= 0x10;
    else    s->raw[11] &= ~0x10;
    s->raw[12] = hitachi_ac1_calc_checksum(s->raw, HITACHI_AC1_STATE_LEN);
}

bool hitachi_ac1_get_power_toggle(const hitachi_ac1_state_t *s)
{
    if (!s) return false;
    return (s->raw[11] & 0x10) != 0;  /* Byte 11, bit 4 = PowerToggle */
}

void hitachi_ac1_set_temp(hitachi_ac1_state_t *s, uint8_t celsius)
{
    if (!s) return;
    /* 官方行為：AUTO 模式下不允許改溫度（自動控制，使用者無法手動設定）
     * 參考 IRremoteESP8266 ir_Hitachi.cpp:600 setTemp() */
    uint8_t mode = hitachi_ac1_get_mode(s);
    if (mode == HITACHI_AC1_MODE_AUTO) return;

    if (celsius < HITACHI_AC1_MIN_TEMP) celsius = HITACHI_AC1_MIN_TEMP;
    if (celsius > HITACHI_AC1_MAX_TEMP) celsius = HITACHI_AC1_MAX_TEMP;

    /* 溫度在 Byte 6 的 bits 2-6 (5 bits, LSB order，需 reverse 5 bits) */
    uint8_t temp_val = celsius - HITACHI_AC1_TEMP_DELTA;
    s->raw[6] = (s->raw[6] & 0x83) | (reverse_5bits(temp_val) << 2);
    s->raw[12] = hitachi_ac1_calc_checksum(s->raw, HITACHI_AC1_STATE_LEN);
}

uint8_t hitachi_ac1_get_temp(const hitachi_ac1_state_t *s)
{
    if (!s) return 0;
    /* Temp 是 5 bits (bits 2-6 of byte 6), 需要 reverse 5 bits */
    uint8_t temp_val = (s->raw[6] >> 2) & 0x1F;
    return reverse_5bits(temp_val) + HITACHI_AC1_TEMP_DELTA;
}

void hitachi_ac1_set_mode(hitachi_ac1_state_t *s, uint8_t mode)
{
    if (!s) return;
    /* 官方行為：AUTO 模式時先設溫度為 25°C（kHitachiAc1TempAuto）
     * 無效 mode 預設為 AUTO + setTemp(25)
     * 參考 IRremoteESP8266 ir_Hitachi.cpp:571-589 setMode() */
    uint8_t new_mode = mode;
    switch (mode) {
    case HITACHI_AC1_MODE_AUTO:
        hitachi_ac1_set_temp(s, HITACHI_AC1_TEMP_AUTO);
        /* FALL THRU — setTemp 在舊 mode 下執行（若舊 mode 是 AUTO 則 setTemp return early） */
    case HITACHI_AC1_MODE_FAN:
    case HITACHI_AC1_MODE_HEAT:
    case HITACHI_AC1_MODE_COOL:
    case HITACHI_AC1_MODE_DRY:
        break;
    default:
        hitachi_ac1_set_temp(s, HITACHI_AC1_TEMP_AUTO);
        new_mode = HITACHI_AC1_MODE_AUTO;
        break;
    }
    /* Mode 在 Byte 5 的高 4 bits */
    s->raw[5] = (s->raw[5] & 0x0F) | ((new_mode << 4) & 0xF0);
    /* 模式變更後重新驗證 fan speed（如 DRY→Low, AUTO→Auto） */
    hitachi_ac1_set_fan(s, hitachi_ac1_get_fan(s));
    s->raw[12] = hitachi_ac1_calc_checksum(s->raw, HITACHI_AC1_STATE_LEN);
}

uint8_t hitachi_ac1_get_mode(const hitachi_ac1_state_t *s)
{
    if (!s) return 0;
    return (s->raw[5] >> 4) & 0x0F;
}

void hitachi_ac1_set_fan(hitachi_ac1_state_t *s, uint8_t speed)
{
    if (!s) return;
    /* 官方行為：依目前 mode 限制 fan speed
     * 參考 IRremoteESP8266 ir_Hitachi.cpp:617-642 setFan() */
    uint8_t mode = hitachi_ac1_get_mode(s);
    uint8_t new_fan;
    switch (mode) {
    case HITACHI_AC1_MODE_DRY:
        new_fan = HITACHI_AC1_FAN_LOW;  /* DRY 鎖定 Low */
        break;
    case HITACHI_AC1_MODE_AUTO:
        new_fan = HITACHI_AC1_FAN_AUTO;  /* AUTO 鎖定 Auto */
        break;
    case HITACHI_AC1_MODE_HEAT:
    case HITACHI_AC1_MODE_FAN:
        /* HEAT/FAN 不允許 Auto：若請求 Auto 或目前是 Auto，強制 Low */
        if (speed == HITACHI_AC1_FAN_AUTO ||
            (s->raw[5] & 0x0F) == HITACHI_AC1_FAN_AUTO) {
            new_fan = HITACHI_AC1_FAN_LOW;
        } else {
            new_fan = s->raw[5] & 0x0F;  /* 保持目前 fan（不接受新值） */
        }
        break;
    default:
        /* COOL 或未知 mode：接受有效值，否則預設 Auto */
        switch (speed) {
        case HITACHI_AC1_FAN_AUTO:
        case HITACHI_AC1_FAN_HIGH:
        case HITACHI_AC1_FAN_MED:
        case HITACHI_AC1_FAN_LOW:
            new_fan = speed;
            break;
        default:
            new_fan = HITACHI_AC1_FAN_AUTO;
            break;
        }
        break;
    }
    /* Fan 在 Byte 5 的低 4 bits */
    s->raw[5] = (s->raw[5] & 0xF0) | (new_fan & 0x0F);
    s->raw[12] = hitachi_ac1_calc_checksum(s->raw, HITACHI_AC1_STATE_LEN);
}

uint8_t hitachi_ac1_get_fan(const hitachi_ac1_state_t *s)
{
    if (!s) return 0;
    return s->raw[5] & 0x0F;
}

void hitachi_ac1_set_model(hitachi_ac1_state_t *s, uint8_t model)
{
    if (!s) return;
    /* Model 在 Byte 3 的 bit 6-7 */
    s->raw[3] = (s->raw[3] & 0x3F) | ((model << 6) & 0xC0);
    s->raw[12] = hitachi_ac1_calc_checksum(s->raw, HITACHI_AC1_STATE_LEN);
}

uint8_t hitachi_ac1_get_model(const hitachi_ac1_state_t *s)
{
    if (!s) return 0;
    return (s->raw[3] >> 6) & 0x03;
}

/* ========== 編碼 ========== */
esp_err_t hitachi_ac1_encode(const hitachi_ac1_state_t *state,
                             uint32_t *timings, size_t max_count, size_t *out_count)
{
    if (!state || !timings || !out_count) return ESP_ERR_INVALID_ARG;

    /* 需要：header(2) + 104 bits*2 + footer mark(1) + gap(1) = 212 */
    const size_t needed = 2 + HITACHI_AC1_BITS * 2 + 1 + 1;
    if (max_count < needed) return ESP_ERR_NO_MEM;

    size_t idx = 0;

    /* Header */
    timings[idx++] = HITACHI_AC1_HDR_MARK;
    timings[idx++] = HITACHI_AC1_HDR_SPACE;

    /* Data: MSB first, 13 bytes = 104 bits */
    for (int byte = 0; byte < HITACHI_AC1_STATE_LEN; byte++) {
        uint8_t b = state->raw[byte];
        for (int bit = 7; bit >= 0; bit--) {
            /* Bit mark */
            timings[idx++] = HITACHI_AC1_BIT_MARK;
            /* Space: 1 or 0 */
            if ((b >> bit) & 1) {
                timings[idx++] = HITACHI_AC1_ONE_SPACE;
            } else {
                timings[idx++] = HITACHI_AC1_ZERO_SPACE;
            }
        }
    }

    /* Footer mark */
    timings[idx++] = HITACHI_AC1_BIT_MARK;
    /* Gap */
    timings[idx++] = HITACHI_AC1_GAP;

    *out_count = idx;
    return ESP_OK;
}

/* ========== 解碼 ========== */
esp_err_t hitachi_ac1_decode(const uint32_t *timings, size_t count,
                             hitachi_ac1_state_t *state)
{
    if (!timings || !state) return ESP_ERR_INVALID_ARG;

    /* 期望：header(2) + 104 bits*2 + footer mark(1) = 211 timings */
    const size_t expected = 2 + HITACHI_AC1_BITS * 2 + 1;
    if (count < expected) {
        ESP_LOGW(TAG, "too short: %u < %u", (unsigned)count, (unsigned)expected);
        return ESP_ERR_INVALID_ARG;
    }

    /* 驗證 header */
    const int hdr_mark_tol = HITACHI_AC1_HDR_MARK * 0.25;
    const int hdr_space_tol = HITACHI_AC1_HDR_SPACE * 0.25;
    if (timings[0] < HITACHI_AC1_HDR_MARK - hdr_mark_tol ||
        timings[0] > HITACHI_AC1_HDR_MARK + hdr_mark_tol) {
        ESP_LOGW(TAG, "bad hdr_mark: %u (expect %u)", (unsigned)timings[0], HITACHI_AC1_HDR_MARK);
        return ESP_ERR_INVALID_VERSION;
    }
    if (timings[1] < HITACHI_AC1_HDR_SPACE - hdr_space_tol ||
        timings[1] > HITACHI_AC1_HDR_SPACE + hdr_space_tol) {
        ESP_LOGW(TAG, "bad hdr_space: %u (expect %u)", (unsigned)timings[1], HITACHI_AC1_HDR_SPACE);
        return ESP_ERR_INVALID_VERSION;
    }

    /* 解碼 data：MSB first
     * HITACHI_AC1 的 data 區段結構嚴格：交替的 mark(400us) + space(500/1250us)。
     * 這裡用 mark/space 對齊方式解碼：每個 bit = 一個 mark + 一個 space。
     * 當 mark 不在容忍範圍內，不 continue（會導致後續全部錯位），
     * 而是仍嘗試讀取後面的 space 來維持對齊。 */
    memset(state->raw, 0, HITACHI_AC1_STATE_LEN);
    size_t bit_idx = 0;
    const int bit_mark_tol = HITACHI_AC1_BIT_MARK * 0.30;
    const int one_space_tol = HITACHI_AC1_ONE_SPACE * 0.30;
    const int zero_space_tol = HITACHI_AC1_ZERO_SPACE * 0.30;

    /* data 從 index 2 開始（index 0=hdr_mark, 1=hdr_space），每個 bit 佔 2 timings */
    for (size_t i = 2; i + 1 < count && bit_idx < HITACHI_AC1_BITS; i += 2) {
        uint32_t mark = timings[i];
        uint32_t space = timings[i + 1];

        /* mark 容忍範圍外 → 可能是最後的 footer mark，結束解碼 */
        if (mark < HITACHI_AC1_BIT_MARK - bit_mark_tol ||
            mark > HITACHI_AC1_BIT_MARK + bit_mark_tol) {
            ESP_LOGW(TAG, "mark oob at t[%u]=%u (expect %u±%u), decoded %u bits",
                     (unsigned)i, (unsigned)mark, HITACHI_AC1_BIT_MARK, bit_mark_tol, (unsigned)bit_idx);
            break;
        }

        int bit_val = -1;
        if (space >= HITACHI_AC1_ONE_SPACE - one_space_tol &&
            space <= HITACHI_AC1_ONE_SPACE + one_space_tol) {
            bit_val = 1;
        } else if (space >= HITACHI_AC1_ZERO_SPACE - zero_space_tol &&
                   space <= HITACHI_AC1_ZERO_SPACE + zero_space_tol) {
            bit_val = 0;
        }

        if (bit_val >= 0) {
            int byte_idx = bit_idx / 8;
            int bit_pos = 7 - (bit_idx % 8);
            if (bit_val) {
                state->raw[byte_idx] |= (1 << bit_pos);
            }
            bit_idx++;
        } else if (bit_idx > 0) {
            ESP_LOGD(TAG, "space oob at t[%u]=%u (expect %u or %u), bit %u",
                     (unsigned)(i + 1), (unsigned)space,
                     HITACHI_AC1_ONE_SPACE, HITACHI_AC1_ZERO_SPACE, (unsigned)bit_idx);
        }
    }

    if (bit_idx < HITACHI_AC1_BITS) {
        ESP_LOGW(TAG, "incomplete: %u bits < %u", (unsigned)bit_idx, HITACHI_AC1_BITS);
        return ESP_ERR_INVALID_ARG;
    }

    /* 驗證 checksum */
    if (!hitachi_ac1_check_checksum(state->raw, HITACHI_AC1_STATE_LEN)) {
        uint8_t calc = hitachi_ac1_calc_checksum(state->raw, HITACHI_AC1_STATE_LEN);
        ESP_LOGW(TAG, "checksum mismatch: calc=0x%02X recv=0x%02X",
                 (unsigned)calc, (unsigned)state->raw[12]);
        return ESP_FAIL;
    }

    /* 驗證 mode 和 fan 是否為有效值（過濾雜訊） */
    uint8_t mode = (state->raw[5] >> 4) & 0x0F;
    uint8_t fan = state->raw[5] & 0x0F;
    if (mode != HITACHI_AC1_MODE_AUTO && mode != HITACHI_AC1_MODE_COOL &&
        mode != HITACHI_AC1_MODE_HEAT && mode != HITACHI_AC1_MODE_FAN) {
        ESP_LOGW(TAG, "invalid mode: 0x%X", (unsigned)mode);
        return ESP_ERR_INVALID_VERSION;
    }
    if (fan != HITACHI_AC1_FAN_AUTO && fan != HITACHI_AC1_FAN_LOW &&
        fan != HITACHI_AC1_FAN_MED && fan != HITACHI_AC1_FAN_HIGH) {
        ESP_LOGW(TAG, "invalid fan: 0x%X", (unsigned)fan);
        return ESP_ERR_INVALID_VERSION;
    }

    return ESP_OK;
}

/* ========== State log ========== */
void hitachi_ac1_state_log(const hitachi_ac1_state_t *state)
{
    if (!state) return;

    uint8_t power = hitachi_ac1_get_power(state);
    uint8_t temp = hitachi_ac1_get_temp(state);
    uint8_t mode = hitachi_ac1_get_mode(state);
    uint8_t fan = hitachi_ac1_get_fan(state);
    uint8_t model = hitachi_ac1_get_model(state);

    const char *mode_str = "UNK";
    switch (mode) {
        case HITACHI_AC1_MODE_AUTO: mode_str = "AUTO"; break;
        case HITACHI_AC1_MODE_COOL: mode_str = "COOL"; break;
        case HITACHI_AC1_MODE_HEAT: mode_str = "HEAT"; break;
        case HITACHI_AC1_MODE_DRY:  mode_str = "DRY";   break;
        case HITACHI_AC1_MODE_FAN:  mode_str = "FAN";   break;
    }

    const char *fan_str = "UNK";
    switch (fan) {
        case HITACHI_AC1_FAN_AUTO: fan_str = "AUTO"; break;
        case HITACHI_AC1_FAN_LOW:  fan_str = "LOW";  break;
        case HITACHI_AC1_FAN_MED:  fan_str = "MED";  break;
        case HITACHI_AC1_FAN_HIGH: fan_str = "HIGH"; break;
    }

    const char *model_str = "UNK";
    if (model == HITACHI_AC1_MODEL_A) model_str = "A";
    else if (model == HITACHI_AC1_MODEL_B) model_str = "B";

    ESP_LOGI(TAG, "State: Model=%s Pwr=%s PwrTgl=%s Mode=%s Temp=%uC Fan=%s",
             model_str,
             power ? "ON" : "OFF",
             hitachi_ac1_get_power_toggle(state) ? "Y" : "N",
             mode_str, (unsigned)temp, fan_str);
    ESP_LOGI(TAG, "  Raw: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             state->raw[0], state->raw[1], state->raw[2], state->raw[3],
             state->raw[4], state->raw[5], state->raw[6], state->raw[7],
             state->raw[8], state->raw[9], state->raw[10], state->raw[11],
             state->raw[12]);
    ESP_LOGI(TAG, "  Byte11=0x%02X (bit5=Pwr=%d, bit4=PwrTgl=%d)",
             state->raw[11],
             (state->raw[11] >> 5) & 0x01,
             (state->raw[11] >> 4) & 0x01);
}
