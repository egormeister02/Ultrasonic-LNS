#pragma once
//
// Платформенный слой decadriver для ESP32-S3: инициализация SPI и сброс DWM1000.
// Функции readfromspi/writetospi/decamutexon/decamutexoff/deca_sleep, которых ждёт
// вендорский deca_device.c, реализованы в deca_glue.c.
//
#include "esp_err.h"

esp_err_t deca_spi_init(void);   // настроить аппаратный SPI (FSPI, mode0, ручной CS, 2 МГц) и сбросить модуль RSTn
void      deca_reset(void);      // повторный аппаратный сброс RSTn + прайм (для ретрая инициализации на холодном старте)
void      deca_spi_diag(void);   // чтение DEV_ID + round-trip записи PANADR (диагностика R/W SPI)
uint32_t  deca_read_devid(void); // однократное чтение DEV_ID для живого мониторинга в цикле
