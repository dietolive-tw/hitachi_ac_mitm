/*
 * usb_jtag_console.cpp - USB Serial/JTAG console input task
 *
 * 讀取 /dev/ttyACM0 (USB Serial/JTAG) 輸入，轉發給 esp_console 執行。
 * 不使用 linenoise（Matter shell 的 linenoise 綁定 UART0 stdin），
 * 而是直接用 usb_serial_jtag driver 讀取字元。
 *
 * 輸出仍走 ROM 直接路徑（esp_rom_usb_serial_putc），不需要 driver；
 * 只有輸入需要 driver 來接收 host 端送來的字元。
 * 為了避免 driver install 後 ROM output 與 driver TX 衝突，
 * 安裝 driver 後也把 console output channel 改為 driver write。
 */

#include "esp_log.h"
#include "esp_console.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#include "driver/usb_serial_jtag_vfs.h"
#else
#include "esp_vfs_usb_serial_jtag.h"
#endif

static const char *TAG = "usj_console";

static SemaphoreHandle_t s_console_mux = NULL;
static bool s_driver_installed = false;

/* 自訂 console putc：透過 driver 寫出，避免與 ROM direct output 衝突 */
static void usj_console_putc(char c)
{
    if (s_driver_installed) {
        usb_serial_jtag_write_bytes((const uint8_t *)&c, 1, 0);
    }
}

static void usj_console_task(void *arg)
{
    /* 安裝 USB Serial/JTAG driver */
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "driver install failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    s_driver_installed = true;

    /* 將 ROM console channel 1 (stdout) 改為透過 driver 輸出 */
    /* channel 1 = stdout, channel 2 = stderr */
    extern void esp_rom_install_channel_putc(int channel, void (*putc)(char c));
    esp_rom_install_channel_putc(1, usj_console_putc);
    esp_rom_install_channel_putc(2, usj_console_putc);

    ESP_LOGI(TAG, "USB Serial/JTAG console ready (send commands via /dev/ttyACM0)");

    static char line[512];
    size_t line_len = 0;
    uint8_t buf[64];

    while (true) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\r' || c == '\n') {
                if (line_len > 0) {
                    line[line_len] = '\0';
                    /* 執行命令 */
                    if (s_console_mux) {
                        xSemaphoreTake(s_console_mux, portMAX_DELAY);
                    }
                    int ret = 0;
                    esp_err_t r = esp_console_run(line, &ret);
                    if (r != ESP_OK) {
                        printf("Error: %s\r\n", esp_err_to_name(r));
                    } else if (ret != 0) {
                        printf("Error: %d\r\n", ret);
                    } else {
                        printf("Done\r\n");
                    }
                    /*印提示符 */
                    printf("> ");
                    fflush(stdout);
                    if (s_console_mux) {
                        xSemaphoreGive(s_console_mux);
                    }
                    line_len = 0;
                }
            } else if (c == 0x03) {
                /* Ctrl-C: 重置當前行 */
                line_len = 0;
                printf("^C\r\n> ");
                fflush(stdout);
            } else if (c == 0x08 || c == 0x7f) {
                /* Backspace */
                if (line_len > 0) {
                    line_len--;
                    printf("\b \b");
                    fflush(stdout);
                }
            } else if (line_len < sizeof(line) - 1) {
                line[line_len++] = c;
                /* 回音（可選） */
                /* printf("%c", c); fflush(stdout); */
            }
        }
    }
}

esp_err_t usb_jtag_console_start(void)
{
    s_console_mux = xSemaphoreCreateMutex();
    if (!s_console_mux) return ESP_ERR_NO_MEM;

    if (xTaskCreate(usj_console_task, "usj_console", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
