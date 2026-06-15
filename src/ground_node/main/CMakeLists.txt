idf_component_register(SRCS "speed_of_sound.c" "bme280.c" "main.c"
                            "dwm1000.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c decadriver dwm1000_port uwb_protocol)
