//
// TX DEBUG: выдаёт двухимпульсные УЗ-пакеты на 10 Гц без UWB-протокола.
// Назначение: настройка аналогового приёмного тракта.
//
// Стадии 2–3 (handshake/config) ПРОПУЩЕНЫ. Выполняется только:
//   [1] SELFTEST  — DWM1000 SPI + DEV_ID + dwt_initialise. Провал -> фатал.
//   [2] Несущая   — carrier_init: 25 кГц непрерывно на GPIO4/5.
//   [3] ЦИКЛ      — двухимпульсный пакет каждые 100 мс (10 Гц):
//                   импульс 1 — immediate, импульс 2 — delayed +DT_US.
//
// На осциллографе (EXTTXE, пад 10 DWM1000):
//   два чистых строба ~3.1 мс, зазор ~1.4 мс, период 100 мс.
//
#include "config.h"
#include "carrier.h"
#include "dwm1000.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "tx_debug";

static void fault_loop(const char *why)
{
    for (;;) {
        ESP_LOGE(TAG, "ФАТАЛ: %s — проверьте железо/перезагрузите", why);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "TX DEBUG: %u Гц, %u имп/пакет, pulse~%u мкс, gap~%u мкс, DT=%u мкс",
             (unsigned)PACKET_RATE_HZ, (unsigned)PULSES_PER_PKT,
             (unsigned)PULSE_US_NOM, (unsigned)GAP_US_NOM, (unsigned)DT_US);

    if (dwm1000_init() != ESP_OK)
        fault_loop("DWM1000 не отвечает");
    ESP_LOGI(TAG, "[1] DWM1000 OK");

    ESP_ERROR_CHECK(carrier_init());
    carrier_start_freq_trim();   // подстройка частоты делителем 0–3.3 В на GPIO1 (±CARRIER_TRIM_HZ)
    ESP_LOGI(TAG, "[2] несущая %u Гц запущена (подстройка ±%u Гц делителем на GPIO1); начинаем цикл...",
             (unsigned)CARRIER_HZ, (unsigned)CARRIER_TRIM_HZ);

    // Паддинг 0xAA выдерживает нужную длину кадра (= длительность импульса).
    // DATA_PAYLOAD_LEN байт + 2 CRC (аппаратно) -> эфирное время = PULSE_US_NOM.
    uint8_t frame[DATA_PAYLOAD_LEN];
    memset(frame, 0xAA, sizeof(frame));

    uint32_t seq = 0;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        frame[0] = (uint8_t)seq;   // мелкий маркер для лога, на EXTTXE не влияет

        if (!dwm1000_send_immediate(frame, sizeof(frame)))
            ESP_LOGW(TAG, "seq=%u: импульс 1 — нет TXFRS", (unsigned)seq);

#if PULSES_PER_PKT >= 2
        uint64_t t1 = dwm1000_read_txstamp();
        if (!dwm1000_send_delayed(frame, sizeof(frame), t1 + DT_TICKS))
            ESP_LOGW(TAG, "seq=%u: импульс 2 — too late или нет TXFRS", (unsigned)seq);
#endif

        seq++;
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PACKET_PERIOD_MS));
    }
}
