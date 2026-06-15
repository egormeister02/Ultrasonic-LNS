//
// Приёмный (центральный/земляной) узел ground_node: ESP32-S3 + DWM1000 + BME280. Реактивный
// «ведомый» — отвечает передатчику (air_node) по стадиям, без работы вслепую:
//
//   [1] SELFTEST  — проверка DWM1000 (DEV_ID) и BME280 (CHIP_ID + измерение). DWM фатален;
//                   BME без него -> нет скорости звука (предупреждение, рукопожатие тестируем).
//   [2] HANDSHAKE — на PING отвечает PONG: подтверждает, что UWB-канал рабочий.
//   [3] CONFIG    — принимает CONFIG передатчика, ПОДСТРАИВАЕТСЯ под его тайминг (окна с запасом),
//                   шлёт CONFIG_ACK. Дальше air удерживает связь keepalive-CONFIG'ами.
//   [4] RUN       — рабочий цикл: приём DATA + УЗ-тайминг якорей. <-- GATED (в air не запущен).
//
// RESTART-SAFE: узел держит стадию синхронизации `synced` (сбрасывается при перезагрузке узла).
// CONFIG принимается ТОЛЬКО после рукопожатия в текущей сессии; если пришёл CONFIG/DATA, а
// рукопожатия не было (свежий старт после ребута) — отвечаем PONG = сигнал регрессии, по которому
// air опускается к рукопожатию и заново проводит стадии. PING на стадии CONFIG = air перезапустился
// -> сбрасываем synced на рукопожатие. Стадия партнёра кодируется типом его кадра, отдельного поля нет.
//
#include "config.h"
#include "dwm1000.h"
#include "bme280.h"
#include "speed_of_sound.h"
#include "uwb_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "ground_node";

#define WIN_MARGIN_US    1000u   // запас к объявленным длительностям при расчёте окон детекта
#define RX_SLICE_MS       500u   // квант ожидания кадра (между ними — heartbeat)
#define HEARTBEAT_MS     2000u   // как часто напоминать, что узел жив и чего ждёт

// Стадия синхронизации с передатчиком (НЕ на проводе — локальное состояние, сбрасывается ребутом).
typedef enum {
    SY_NONE = 0,    // только включились: рукопожатия в этой сессии не было
    SY_HANDSHAKE,   // PING->PONG пройден, готовы принять CONFIG
    SY_CONFIG,      // CONFIG принят; ждём первый рабочий пакет (стадия 4/RUN — gated)
} gnd_sync_t;

static const char *sync_name(gnd_sync_t s)
{
    switch (s) {
    case SY_NONE:      return "ожидаю PING (стадия 2)";
    case SY_HANDSHAKE: return "ожидаю CONFIG (стадия 3)";
    case SY_CONFIG:    return "жду первый рабочий пакет (стадия 4/RUN — gated)";
    default:           return "?";
    }
}

static void fault_loop(const char *why)
{
    for (;;) {
        ESP_LOGE(TAG, "ФАТАЛ: %s — работа остановлена, проверьте железо/перезагрузите", why);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// [1] Селфтест: DWM1000 (фатально) + BME280 (предупреждение). Возвращает состояние BME.
static bool stage_selftest(void)
{
    if (dwm1000_init() != ESP_OK) fault_loop("селфтест DWM1000 не пройден");

    bool bme_ok = (bme280_init() == ESP_OK);
    bme280_data_t m;
    if (bme_ok && bme280_read(&m) == ESP_OK) {
        double c = speed_of_sound(m.temperature_c + 273.15, m.pressure_pa, m.humidity_rh / 100.0);
        ESP_LOGI(TAG, "[1/4] SELFTEST: DWM1000 OK; BME280 OK (T=%.1f C, p=%.0f Pa, RH=%.0f%% -> c=%.1f м/с)",
                 m.temperature_c, m.pressure_pa, m.humidity_rh, c);
    } else {
        bme_ok = false;
        ESP_LOGE(TAG, "[1/4] SELFTEST: DWM1000 OK; BME280 НЕ НАЙДЕН (I2C SDA=%d SCL=%d адрес 0x%02X) "
                      "-> скорость звука недоступна", I2C_SDA_PIN, I2C_SCL_PIN, BME280_ADDR);
    }
    return bme_ok;
}

void app_main(void)
{
    ESP_LOGI(TAG, "ground_node старт: приёмный/центральный узел");
    bool bme_ok = stage_selftest();
    (void)bme_ok;   // понадобится со стадии 5 (скорость звука для дальности)

    uwb_config_t cfg;                 // принятый конфиг передатчика (для стадии 4/RUN)
    gnd_sync_t synced = SY_NONE;      // стадия синхронизации; ребут узла -> снова SY_NONE
    uint8_t  rx[127];
    uint32_t last_hb = 0;

    // Готовый ответ-регрессия: «я только на рукопожатии» (на out-of-order CONFIG/DATA после ребута).
    const uwb_ping_t pong = { .hdr = { UWB_MAGIC, UWB_VER, UWB_FT_PONG }, .node_id = NODE_ID_GROUND };

    for (;;) {
        int n = dwm1000_recv(rx, sizeof(rx), RX_SLICE_MS);

        // нет кадра (таймаут/ошибка) -> периодический heartbeat: узел жив и на какой стадии ждёт.
        if (n <= 0) {
            uint32_t now = esp_log_timestamp();
            if (now - last_hb >= HEARTBEAT_MS) {
                last_hb = now;
                ESP_LOGI(TAG, "жду air_node: %s | ошибок приёма=%" PRIu32 ", посл. SYS_STATUS=0x%08" PRIX32
                              " (0 ошибок = кадров нет вообще -> питание/антенна/прошивка air)",
                         sync_name(synced), dwm1000_rx_errors(), dwm1000_last_rx_err());
            }
            continue;
        }

        // кадр принят, но не наш протокол/версия (напр. на air залита старая прошивка)
        if (n < (int)sizeof(uwb_hdr_t) || rx[0] != UWB_MAGIC || rx[1] != UWB_VER) {
            ESP_LOGW(TAG, "принят ЧУЖОЙ кадр: len=%d, magic=0x%02X ver=%u (ожидалось 0x%02X/%u) — "
                          "не наш протокол? обнови прошивку air_node",
                     n, rx[0], (n > 1) ? rx[1] : 0, UWB_MAGIC, UWB_VER);
            continue;
        }

        switch (rx[2]) {              // тип кадра
        case UWB_FT_PING: {
            // Рукопожатие. PING на стадии CONFIG = air перезапустился -> сбрасываемся к рукопожатию.
            gnd_sync_t prev = synced;
            synced = SY_HANDSHAKE;
            dwm1000_send((uint8_t *)&pong, sizeof(pong));
            if (prev == SY_CONFIG)
                ESP_LOGW(TAG, "РЕСИНК: PING на стадии CONFIG -> air ПЕРЕЗАПУСТИЛСЯ; сброс к рукопожатию");
            else if (prev != SY_HANDSHAKE)
                ESP_LOGI(TAG, "[2/4] HANDSHAKE: получен PING -> ответил PONG (канал рабочий)");
            break;
        }
        case UWB_FT_CONFIG: {
            if (n < (int)sizeof(uwb_config_t)) {       // битый/короткий конфиг — не регрессия
                uwb_ack_t nak = { .hdr = { UWB_MAGIC, UWB_VER, UWB_FT_CONFIG_ACK },
                                  .node_id = NODE_ID_GROUND, .status = UWB_ERR_CFG };
                dwm1000_send((uint8_t *)&nak, sizeof(nak));
                break;
            }
            if (synced < SY_HANDSHAKE) {                // CONFIG без рукопожатия (свежий старт после ребута)
                dwm1000_send((uint8_t *)&pong, sizeof(pong));   // -> PONG = сигнал регрессии «сначала PING»
                ESP_LOGW(TAG, "CONFIG без рукопожатия (свежий старт) -> ответил PONG, прошу PING");
                break;
            }
            gnd_sync_t prev = synced;
            memcpy(&cfg, rx, sizeof(cfg));              // принять и сохранить тайминг передатчика
            synced = SY_CONFIG;
            uwb_ack_t ack = { .hdr = { UWB_MAGIC, UWB_VER, UWB_FT_CONFIG_ACK },
                              .node_id = NODE_ID_GROUND, .status = UWB_OK };
            dwm1000_send((uint8_t *)&ack, sizeof(ack));
            if (prev != SY_CONFIG) {                    // лог один раз на вход в стадию (не на каждый keepalive)
                // ПОДСТРОЙКА под передатчик: окна детекта = объявленные длительности + запас
                ESP_LOGI(TAG, "[3/4] CONFIG принят: %u Гц, %u имп; имп=%u мкс, зазор=%u мкс, DT=%u мкс, несущая=%u Гц",
                         cfg.packet_rate_hz, cfg.pulses_per_pkt, cfg.pulse_us, cfg.gap_us,
                         cfg.dt_us, cfg.carrier_hz);
                ESP_LOGI(TAG, "        окно импульса ~%u мкс, окно 2-го импульса ~DT±%u мкс (с запасом); CONFIG_ACK отправлен",
                         (unsigned)(cfg.pulse_us + WIN_MARGIN_US), (unsigned)WIN_MARGIN_US);
                ESP_LOGI(TAG, "Удержание линка: air шлёт keepalive-CONFIG. Стадия 4/RUN — gated.");
            }
            break;
        }
        case UWB_FT_DATA:
            // Стадия 4/RUN GATED — рабочих пакетов быть не должно. Если пришёл, значит мы отстали
            // от air (свой ребут) -> PONG = сигнал регрессии, пусть переинициализирует с рукопожатия.
            dwm1000_send((uint8_t *)&pong, sizeof(pong));
            break;
        default:
            break;
        }
    }
}
