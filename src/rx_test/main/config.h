#pragma once
//
// Параметры тестовой прошивки приёмного (земляного) узла: BME280 + скорость звука.
//

// --- I2C (BME280) ---
#define I2C_SDA_PIN    8
#define I2C_SCL_PIN    9
#define I2C_FREQ_HZ    100000
#define BME280_ADDR    0x76      // 0x77 если вывод SDO подтянут к VCC

// Как часто опрашивать датчик и печатать скорость звука, мс
#define MEAS_PERIOD_MS 1000
