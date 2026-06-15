#pragma once
//
// DWM1000 узла ground_node на вендорском decadriver: приём кадров от air_node + передача
// ответов (PONG / CONFIG_ACK). Конфигурация совпадает с передатчиком (air_node):
// ch5/PRF16/PLEN2048/110k/pcode4. Приём попутно подводит XTAL-trim к нулю частотного сдвига.
//
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Селфтест + конфиг: SPI + dwt_initialise(LDE) + DEV_ID==0xDECA0130 + dwt_configure + TXRF.
// ESP_OK только при полностью исправном модуле.
esp_err_t dwm1000_init(void);

// Приём ОДНОГО кадра с программным таймаутом. Возвращает длину полезной нагрузки (без 2 CRC),
// либо 0 при таймауте/ошибке приёма (счётчик ошибок — dwm1000_rx_errors). Попутно тюнит XTAL-trim.
int dwm1000_recv(uint8_t *buf, size_t bufsize, uint32_t timeout_ms);

// Накопленное число ошибок приёма (для диагностики «молчит / шумит / чужой протокол»).
uint32_t dwm1000_rx_errors(void);

// SYS_STATUS последней ошибки приёма (биты: RXPHE/RXFCE=кадр пойман но битый; RXSFDTO=преамбулу
// видит, SFD нет; и т.п.) — отличить шум от реальных, но повреждённых кадров.
uint32_t dwm1000_last_rx_err(void);

// Немедленная передача ответа. Ждёт TXFRS. false при ошибке старта/таймауте.
bool dwm1000_send(const uint8_t *data, size_t len);
