idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES esp_adc
                    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio nvs_flash
                    INCLUDE_DIRS "."
                    SRC_DIRS  "." "../../esp-zigbee-sdk/examples/common/zcl_utility/src"
                    INCLUDE_DIRS "." "../../esp-zigbee-sdk/examples/common/zcl_utility/include"
                    REQUIRES spiffs switch_driver
                    REQUIRES espressif__esp-zigbee-lib espressif__esp-zboss-lib
                    REQUIRES led_strip)
