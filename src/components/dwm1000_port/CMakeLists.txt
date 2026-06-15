# Общий порт-слой DWM1000 для всех узлов: распиновка (dwm1000_pins.h) и платформенный
# интерфейс decadriver (deca_glue.c/.h — АППАРАТНЫЙ SPI mode0, slow/fast клок, сброс RSTn).
# REQUIRES: decadriver (типы и DWT_DEVICE_ID из deca_device_api.h), esp_driver_gpio (RSTn),
#           esp_driver_spi (spi_master).
idf_component_register(SRCS "deca_glue.c"
                       INCLUDE_DIRS "."
                       REQUIRES decadriver esp_driver_gpio esp_driver_spi)
