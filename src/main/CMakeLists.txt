idf_component_register(SRC_DIRS "."
                        INCLUDE_DIRS "."
                        "${CMAKE_CURRENT_SOURCE_DIR}/../esp-zigbee-sdk/components/esp-zigbee-lib/include"
                        PRIV_REQUIRES esp_adc
                        PRIV_REQUIRES nvs_flash esp_driver_uart ieee802154)
