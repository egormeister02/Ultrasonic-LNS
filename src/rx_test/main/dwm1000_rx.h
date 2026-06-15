#pragma once
//
// Приём UWB-кадров через DWM1000 (вендорский decadriver).
// Конфигурация совпадает с передатчиком (src/proto): канал 5, PRF 16 МГц,
// преамбула 1024, 6.8 Мбит/с, preamble code 4.
//
#include "esp_err.h"

esp_err_t dwm1000_rx_init(void);   // SPI + сброс + dwt_initialise(LDE) + dwt_configure
void      dwm1000_rx_start(void);  // запустить FreeRTOS-задачу непрерывного приёма
