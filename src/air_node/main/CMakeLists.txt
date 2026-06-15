idf_component_register(SRCS "main.c"
                            "carrier.c"
                            "dwm1000.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_mcpwm decadriver dwm1000_port uwb_protocol)
