/*
 * Hitachi AC MITM - app_main.cpp
 *
 * 改造自 esp-matter/examples/room_air_conditioner/main/app_main.cpp
 *
 * Matter 裝置架構（3 個 endpoint）：
 *   Endpoint 1: thermostat            (Thermostat + FanControl + TemperatureMeasurement)  ← 冷氣主控制 + 室溫
 *   Endpoint 2: contact_sensor        (BooleanState)                    ← 光敏電阻 = 冷氣電源狀態
 *   Endpoint 3: contact_sensor        (BooleanState)                    ← 壓縮機運轉狀態
 *
 * 對應 ESPHome yaml 的：
 *   - HA Template Climate → thermostat (Thermostat + FanControl)
 *   - sensor.室內溫度 (DS18B20) → temperature_sensor
 *   - binary_sensor.冷氣電源狀態 → contact_sensor #1
 *   - binary_sensor.壓縮機運轉狀態 → contact_sensor #2
 */

#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <setup_payload/OnboardingCodesUtil.h>

/* DS18B20 與 GPIO 監控 driver 的註冊函式（在各自 .cpp 實作） */
extern "C" esp_err_t ds18b20_driver_init(void);
extern void ds18b20_register_update_cb(void (*cb)(float temp_c));
extern esp_err_t gpio_monitor_init(void);
extern void gpio_monitor_register_ac_power_cb(void (*cb)(bool state));
extern void gpio_monitor_register_compressor_cb(void (*cb)(bool state));
extern esp_err_t ir_driver_init(void);

/* app_driver 內部實作（app_driver.cpp） */
extern void app_driver_register_callbacks(void);
extern esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                              uint32_t attribute_id, esp_matter_attr_val_t *val);
extern esp_err_t app_driver_hitachi_ac_set_defaults(uint16_t endpoint_id);
extern app_driver_handle_t app_driver_hitachi_ac_init(void);

/* WS2812B 狀態指示燈（status_led.cpp） */
extern esp_err_t status_led_init(void);
extern void status_led_set_state(status_led_state_t state);
extern void status_led_identify(bool start);

static const char *TAG = "app_main";

/* Endpoint IDs（app_driver.cpp 會用 extern 引用） */
uint16_t room_air_conditioner_endpoint_id = 0;
uint16_t ac_power_contact_endpoint_id = 0;
uint16_t compressor_contact_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

/* NVS namespace 與 key 名稱（與 ESP32Config 內部定義一致） */
static constexpr const char *kChipFactoryNvsNamespace = "chip-factory";
static constexpr const char *kDiscriminatorNvsKey = "discriminator";

/* 從 ESP32-C6 的唯一 MAC 位址衍生 12-bit setup discriminator。
 * 寫入 NVS chip-factory namespace；若已存在值則保留（factory reset 之後才會重算）。
 * Passcode 維持預設 20202021（好記），只需 discriminator 每台不同即可讓 QR code 唯一。 */
static void derive_setup_discriminator_from_mac()
{
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MAC address; using default discriminator");
        return;
    }
    /* 取 MAC 後 12 bits 作為 discriminator（0x000..0xFFF），最後 2 bytes 變化最大 */
    uint16_t derived = ((uint16_t)(mac[4] & 0xFF) << 4) | (uint16_t)(mac[5] >> 4);

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kChipFactoryNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", kChipFactoryNvsNamespace, esp_err_to_name(err));
        return;
    }

    uint32_t existing = 0;
    if (nvs_get_u32(handle, kDiscriminatorNvsKey, &existing) == ESP_OK) {
        ESP_LOGI(TAG, "Setup discriminator already in NVS: 0x%03" PRIX32 " (skip derivation)", existing);
    } else {
        err = nvs_set_u32(handle, kDiscriminatorNvsKey, (uint32_t)derived);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write discriminator: %s", esp_err_to_name(err));
        } else {
            err = nvs_commit(handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to commit discriminator: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "Setup discriminator derived from MAC %02X:%02X:%02X:%02X:%02X:%02X -> 0x%03X",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], derived);
            }
        }
    }
    nvs_close(handle);
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        status_led_set_state(STATUS_LED_CONNECTED);
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        status_led_set_state(STATUS_LED_COMMISSIONING);
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        status_led_set_state(STATUS_LED_COMMISSIONING);
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        /* 若已加入 fabric 則顯示已連線，否則回到配對中（window 關閉且無 fabric 時會重開） */
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0) {
            status_led_set_state(STATUS_LED_CONNECTED);
        }
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved: {
        ESP_LOGI(TAG, "Fabric removed successfully");
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            chip::CommissioningWindowManager &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
            if (!commissionMgr.IsCommissioningWindowOpen()) {
                CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                                            chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err != CHIP_NO_ERROR) {
                    ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                }
            }
        }
        break;
    }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    /* 驅動 WS2812B LED 識別效果：START/STOP 切換黃色閃爍 */
    if (type == identification::START) {
        status_led_identify(true);
    } else if (type == identification::STOP) {
        status_led_identify(false);
    }
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* 在 Matter 啟動前先把 setup discriminator 寫入 NVS（從 MAC 衍生，每台不同） */
    derive_setup_discriminator_from_mac();

    /* 初始化 WS2812B 狀態指示燈（最早初始化，開機即顯示配對中藍色閃爍） */
    status_led_init();

    /* Initialize hardware drivers */
    app_driver_handle_t ac_handle = app_driver_hitachi_ac_init();
    app_driver_handle_t button_handle = app_driver_button_init();
    app_reset_button_register(button_handle);

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    /* ===== Endpoint 1: Thermostat + FanControl (冷氣主控制) =====
     * 不用 room_air_conditioner::create()（強制含 OnOff cluster），改用 thermostat::create()
     * 電源開關由 SystemMode=Off/Cool/Heat 控制，不另設 OnOff switch entity */
    thermostat::config_t thermo_cfg;
    thermo_cfg.thermostat.local_temperature = nullable<int16_t>(DEFAULT_LOCAL_TEMP_C * 100);
    thermo_cfg.thermostat.system_mode = (uint8_t)Thermostat::SystemModeEnum::kCool;
    thermo_cfg.thermostat.control_sequence_of_operation = 4; /* Cooling only */
    thermo_cfg.thermostat.feature_flags |= cluster::thermostat::feature::cooling::get_id();
    endpoint_t *rac_ep = thermostat::create(node, &thermo_cfg, ENDPOINT_FLAG_NONE, ac_handle);
    ABORT_APP_ON_FAILURE(rac_ep != nullptr, ESP_LOGE(TAG, "Failed to create thermostat endpoint"));

    /* 新增 Fan (0x002B) 為 secondary device type — HomeKit 需要 endpoint 同時宣告
     * Thermostat + Fan 才能正確顯示溫度轉盤 + 風扇控制 UI。
     * 只註冊 device type ID，不重複建立 cluster（Identify/Groups/FanControl 已由
     * thermostat::create 和後續手動建立提供） */
    esp_err_t dt_err = endpoint::add_device_type(rac_ep, 0x002B, 4);
    ABORT_APP_ON_FAILURE(dt_err == ESP_OK, ESP_LOGE(TAG, "Failed to add Fan device type to endpoint"));

    /* 新增 Temperature Sensor (0x0302) 為 secondary device type — HomeKit 看 device
     * type 決定 service 對應，不是看 cluster。雖然 EP1 已有 TemperatureMeasurement
     * cluster，但沒有 0x0302 device type 宣告，HomeKit 不會顯示溫度感測器 service。
     * 0x0302 的 server clusters = Identify + Groups(optional) + TemperatureMeasurement
     * 都已在 EP1 提供。 */
    dt_err = endpoint::add_device_type(rac_ep, 0x0302, 4);
    ABORT_APP_ON_FAILURE(dt_err == ESP_OK, ESP_LOGE(TAG, "Failed to add Temp Sensor device type"));

    room_air_conditioner_endpoint_id = endpoint::get_id(rac_ep);
    ESP_LOGI(TAG, "Thermostat endpoint created, endpoint_id=%d", room_air_conditioner_endpoint_id);

    /* FanControl cluster（co-located with Thermostat）
     * FanMode: 0=Off, 1=Low, 2=Medium, 3=High, 4=On, 5=Auto, 6=Smart
     * fanModeSequence=2 (Low/Medium/High/Off) */
    cluster::fan_control::config_t fan_cfg;
    fan_cfg.fan_mode = (uint8_t)FanControl::FanModeEnum::kLow;
    fan_cfg.fan_mode_sequence = 2;  /* Off/Low/Medium/High */
    cluster_t *fan_cluster = cluster::fan_control::create(rac_ep, &fan_cfg, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(fan_cluster != nullptr, ESP_LOGE(TAG, "Failed to create FanControl cluster"));

    /* TemperatureMeasurement cluster（co-located with Thermostat + FanControl）
     * DS18B20 室溫同時寫入此 cluster 的 measured_value 和 Thermostat.local_temperature */
    cluster::temperature_measurement::config_t tm_cfg;
    tm_cfg.measured_value = nullable<int16_t>(DEFAULT_LOCAL_TEMP_C * 100);
    tm_cfg.min_measured_value = nullable<int16_t>(-5500);
    tm_cfg.max_measured_value = nullable<int16_t>(12500);
    cluster_t *tm_cluster = cluster::temperature_measurement::create(rac_ep, &tm_cfg, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(tm_cluster != nullptr, ESP_LOGE(TAG, "Failed to create TemperatureMeasurement cluster"));

    /* ===== Endpoint 2: Contact Sensor - 冷氣電源狀態 (光敏電阻) ===== */
    /* 對應 ESPHome: binary_sensor.冷氣電源狀態 */
    contact_sensor::config_t ac_power_cfg;
    /* contact_sensor 繼承 app_with_bool_state_config，內含 boolean_state cluster */
    endpoint_t *ac_power_ep = contact_sensor::create(node, &ac_power_cfg, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(ac_power_ep != nullptr, ESP_LOGE(TAG, "Failed to create ac_power contact_sensor endpoint"));
    ac_power_contact_endpoint_id = endpoint::get_id(ac_power_ep);
    ESP_LOGI(TAG, "AC Power Contact Sensor created, endpoint_id=%d", ac_power_contact_endpoint_id);

    /* ===== Endpoint 3: Contact Sensor - 壓縮機運轉狀態 ===== */
    /* 對應 ESPHome: binary_sensor.壓縮機運轉狀態 */
    contact_sensor::config_t comp_cfg;
    endpoint_t *comp_ep = contact_sensor::create(node, &comp_cfg, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(comp_ep != nullptr, ESP_LOGE(TAG, "Failed to create compressor contact_sensor endpoint"));
    compressor_contact_endpoint_id = endpoint::get_id(comp_ep);
    ESP_LOGI(TAG, "Compressor Contact Sensor created, endpoint_id=%d", compressor_contact_endpoint_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CHIP_DEVICE_CONFIG_ENABLE_WIFI_STATION
    /* WiFi+Thread 雙模：建立 secondary network interface endpoint（給 Thread network commissioning） */
    /* endpoint id 由 CONFIG_THREAD_NETWORK_ENDPOINT_ID 決定（預設 2），不能和 WiFi 的 endpoint 0 衝突 */
    secondary_network_interface::config_t secondary_network_interface_config;
    endpoint_t *secondary_net_ep =
        endpoint::secondary_network_interface::create(node, &secondary_network_interface_config, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(secondary_net_ep != nullptr, ESP_LOGE(TAG, "Failed to create secondary network interface endpoint"));
    ESP_LOGI(TAG, "Secondary network interface (Thread) created, endpoint_id=%d", endpoint::get_id(secondary_net_ep));
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    /* 檢查 fabric 狀態：無 fabric 時顯示配對中（藍色閃爍），有 fabric 時顯示已連線（綠色） */
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
        status_led_set_state(STATUS_LED_COMMISSIONING);
    } else {
        status_led_set_state(STATUS_LED_CONNECTED);
    }

    /* 輸出配對 QR Code 與 Manual Pairing Code（BLE rendezvous） */
    PrintOnboardingCodes(chip::RendezvousInformationFlag(chip::RendezvousInformationFlag::kBLE));

    /* Starting driver with default values */
    app_driver_hitachi_ac_set_defaults(room_air_conditioner_endpoint_id);

    /* 註冊硬體狀態 → Matter attribute 的 callback */
    app_driver_register_callbacks();

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::attribute_register_commands();
    esp_matter::console::init();
    ir_console_init();
    app_driver_console_init();
    gpio_monitor_console_init();
    /* 啟動 USB Serial/JTAG console input task（透過 /dev/ttyACM0 下指令） */
    usb_jtag_console_start();
#endif
}
