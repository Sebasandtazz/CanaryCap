idf_component_register(
    SRCS "main.c"
    SRC_DIRS "." "../../esp-zigbee-sdk/examples/common/zcl_utility/src" "../../esp-zigbee-sdk/examples/common/switch_driver"
    INCLUDE_DIRS "." "../../esp-zigbee-sdk/examples/common/zcl_utility/include"
    PRIV_REQUIRES esp_adc esp_driver_ledc esp_driver_gpio nvs_flash esp_timer driver
    REQUIRES spiffs switch_driver esp-zigbee-lib esp-zboss-lib led_strip
)
