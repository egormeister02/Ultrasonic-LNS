#pragma once
//
// DWM1000 узла air_node на вендорском decadriver: передача УЗ-синхро + приём ответов земли.
//
// GPIO5/EXTTXE включается через dwt_setlnapamode(0,1) + fine-grain TX-seq off: на время передачи
// каждого UWB-кадра пин одним чистым стробом открывает ключи SN74HC4066. Двухимпульсная форма —
// две передачи с точным разносом ΔT через Delayed-TX (dwt_setdelayedtrxtime). Конфигурация
// совпадает с приёмником (ground_node): ch5/PRF16/PLEN2048/110k/pcode4.
//
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Селфтест + конфиг: SPI + dwt_initialise(LDE) + DEV_ID==0xDECA0130 + dwt_configure + EXTTXE.
// ESP_OK только при полностью исправном модуле (иначе наверху — фатал-стейт).
esp_err_t dwm1000_init(void);

// Немедленная передача кадра (EXTTXE поднимается со стартом). Ждёт TXFRS.
bool dwm1000_send_immediate(const uint8_t *data, size_t len);

// Передача в момент dx_time (полные 40 бит сист. времени). EXTTXE поднимается тогда же.
// false при «опоздании» (DWT_ERROR) или таймауте.
bool dwm1000_send_delayed(const uint8_t *data, size_t len, uint64_t dx_time);

uint64_t dwm1000_read_txstamp(void);   // 40-бит метка времени последней передачи

// Приём одного кадра с программным таймаутом (рукопожатие). Возвращает длину полезной нагрузки
// (без 2 CRC), 0 при таймауте/ошибке.
int dwm1000_recv(uint8_t *buf, size_t bufsize, uint32_t timeout_ms);
