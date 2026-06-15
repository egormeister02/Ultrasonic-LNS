idf_component_register(SRCS "main.c"
                            "bme280.c"
                            "speed_of_sound.c"
                            "dwm1000_rx.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c decadriver dwm1000_port)
