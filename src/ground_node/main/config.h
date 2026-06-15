#pragma once
//
// Параметры приёмного (центрального/земляного) узла ground_node.
// Пины DWM1000 (SPI/RST/IRQ) — общие, берутся из dwm1000_pins.h (компонент dwm1000_port).
//

#define NODE_ID_GROUND   2u   // идентификатор приёмного узла (в кадрах PONG/CONFIG_ACK)

// DEBUG_DWM: 1 = подробная SPI-диагностика при инициализации. Продакшен = 0.
#define DEBUG_DWM        0

// --- I2C (BME280: температура/давление/влажность -> скорость звука) ---
#define I2C_SDA_PIN    8
#define I2C_SCL_PIN    9
#define I2C_FREQ_HZ    100000
#define BME280_ADDR    0x76      // 0x77 если вывод SDO подтянут к VCC
