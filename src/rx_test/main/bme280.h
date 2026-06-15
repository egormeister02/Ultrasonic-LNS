#pragma once
//
// Минимальный драйвер BME280 (I2C) для измерения температуры, давления и влажности —
// входные величины для расчёта скорости звука.
//
#include "esp_err.h"

#define BME280_CHIP_ID  0x60   // регистр 0xD0 у BME280

typedef struct {
    float temperature_c;   // °C
    float pressure_pa;     // Па
    float humidity_rh;     // % отн. влажности (0..100)
} bme280_data_t;

// Поднять I2C-шину, проверить CHIP_ID, прочитать калибровку, запустить normal mode.
esp_err_t bme280_init(void);

// Прочитать и скомпенсировать одно измерение.
esp_err_t bme280_read(bme280_data_t *out);
