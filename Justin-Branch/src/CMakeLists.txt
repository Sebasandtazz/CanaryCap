idf_component_register(SRCS "continuous_read_main_without_spiff.c" "PWM_buzzer.c" "continuous_read_main.c"
                    PRIV_REQUIRES esp_adc
                    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio
                    INCLUDE_DIRS "."
                    REQUIRES spiffs)
