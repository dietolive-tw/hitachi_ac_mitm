idf_component_register(SRCS
                       app_main.cpp
                       app_driver.cpp
                       ir_driver.cpp
                       ir_hitachi_ac1.cpp
                       ds18b20_driver.cpp
                       gpio_monitor.cpp
                       status_led.cpp
                       usb_jtag_console.cpp
                       PRIV_INCLUDE_DIRS  "." "${ESP_MATTER_PATH}/examples/common/utils")

target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
