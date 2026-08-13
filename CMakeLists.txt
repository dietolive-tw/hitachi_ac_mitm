# Hitachi AC MITM - Matter Room Air Conditioner + 監控感測器
# 改造自 esp-matter/examples/room_air_conditioner
cmake_minimum_required(VERSION 3.5)

if(NOT DEFINED ENV{ESP_MATTER_PATH})
    message(FATAL_ERROR "Please set ESP_MATTER_PATH to the path of esp-matter repo")
endif(NOT DEFINED ENV{ESP_MATTER_PATH})

if(NOT DEFINED ENV{ESP_MATTER_DEVICE_PATH})
    if("${IDF_TARGET}" STREQUAL "esp32c6")
        set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/esp32c6_devkit_c)
    else()
        message(FATAL_ERROR "This example only supports esp32c6. Unsupported IDF_TARGET: ${IDF_TARGET}")
    endif()
endif(NOT DEFINED ENV{ESP_MATTER_DEVICE_PATH})

# Software version：可用 -DPROJECT_VER_NUMBER=N -DPROJECT_VER="N.X" 覆蓋
# OTA 流程：build 時 bump 版號 → 產生 OTA image → 裝置 (舊版) 接受並更新
# 注意：裝置 SoftwareVersion 屬性 = PROJECT_VER_NUMBER（自動套用）
if(NOT DEFINED PROJECT_VER)
    set(PROJECT_VER "1.0")
endif()
if(NOT DEFINED PROJECT_VER_NUMBER)
    set(PROJECT_VER_NUMBER 1)
endif()
message(STATUS "Hitachi AC MITM: PROJECT_VER=${PROJECT_VER} PROJECT_VER_NUMBER=${PROJECT_VER_NUMBER}")

set(ESP_MATTER_PATH $ENV{ESP_MATTER_PATH})
set(MATTER_SDK_PATH ${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip)

# This should be done before using the IDF_TARGET variable.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include($ENV{ESP_MATTER_DEVICE_PATH}/esp_matter_device.cmake)
include($ENV{ESP_MATTER_PATH}/examples/common/cmake_common/components_include.cmake)

# DS18B20 改用 GPIO bit-bang，不再需要 onewire_bus 元件
set(EXTRA_COMPONENT_DIRS
    "${ESP_MATTER_PATH}/examples/common"
    "${MATTER_SDK_PATH}/config/esp32/components"
    "${ESP_MATTER_PATH}/components"
    "${ESP_MATTER_PATH}/device_hal/device"
    ${extra_components_dirs_append})

project(hitachi_ac_mitm)

if(CONFIG_IDF_TARGET_ESP32C2)
    include(relinker)
endif()

idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security" APPEND)

# Override default TEST_VENDOR/TEST_PRODUCT with custom device identity.
# These macros are #ifndef-guarded in CHIPDeviceConfig.h, so defining them
# globally takes precedence. Used by GenericDeviceInstanceInfoProvider
# (CONFIG_EXAMPLE_DEVICE_INSTANCE_INFO_PROVIDER=y) to serve BasicInformation
# cluster VendorName(0x0001)/ProductName(0x0003) attributes on the root endpoint.
# Note: must be set globally (not just on main component) because the macros
# are consumed in esp_matter / CHIP SDK components via GenericDeviceInstanceInfoProvider.ipp.
idf_build_set_property(COMPILE_OPTIONS "-DCHIP_DEVICE_CONFIG_DEVICE_VENDOR_NAME=\"Ken\";-DCHIP_DEVICE_CONFIG_DEVICE_PRODUCT_NAME=\"DIY_Matter\"" APPEND)
