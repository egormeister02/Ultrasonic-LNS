//
// Передающий узел БПЛА (air_node): ESP32-S3 + DWM1000. Ведущий: инициирует обмен и задаёт тайминг.
// Запуск идёт СТРОГО по стадиям, без работы вслепую:
//
//   [1] SELFTEST  — проверка своего DWM1000 (DEV_ID + dwt_initialise). Провал -> фатал-стейт.
//   [2] HANDSHAKE — PING -> ждём PONG: UWB-канал с землёй в принципе рабочий.
//   [3] CONFIG    — шлём свой тайминг, ждём CONFIG_ACK; затем УДЕРЖАНИЕ с keepalive (см. ниже).
//   [4] RUN       — рабочий цикл: флаш -> бутстрап-выстрел -> УЗ-пары + DATA (run_task). <-- GATED.
//
// RESTART-SAFE (ресинк по стадиям): после CONFIG_ACK НЕ висим вечно, а гоняем keepalive — периодически
// шлём CONFIG. Земля синхронна -> отвечает CONFIG_ACK; земля перезапустилась -> отвечает PONG (она ещё
// не прошла рукопожатие в новой сессии) -> мы это видим как регрессию стадии и возвращаемся к [2]. Если
// земля молчит N keepalive подряд (перезагрузка/потеря) -> тоже ресинк с [2]. Инициатор подъёма всегда
// air, поэтому взаимная блокировка исключена. Стадия партнёра кодируется ТИПОМ его ответа (поля нет).
//
// !!! ГЕЙТ: в этой сборке после стадии 3 уходим в УДЕРЖАНИЕ и НЕ входим в RUN — проверяем restart-safe.
//
// Несущую (carrier_init) запускаем только в рабочем цикле — поэтому в стадиях 1–3 EXTTXE тогглит,
// но при выключенной несущей звука нет (на пьезо ничего не проходит).
//
#include "config.h"
#include "carrier.h"
#include "dwm1000.h"
#include "uwb_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "air_node";

#define HS_REPLY_TIMEOUT_MS   50u    // сколько ждать ответа земли после запроса
#define HS_RETRY_PERIOD_MS    50u    // пауза между попытками рукопожатия/конфига
#define HOLD_KEEPALIVE_MS    300u    // период keepalive-CONFIG в удержании (стадия 3 пройдена)
#define HOLD_MAX_MISSES       10u    // молчаний земли подряд в удержании -> ресинк с рукопожатия

_Static_assert(DATA_PAYLOAD_LEN >= sizeof(uwb_data_t),
               "DATA_PAYLOAD_LEN должен вмещать шапку uwb_data_t (8 байт)");

// Проверка принятого кадра: длина, magic, ver, тип.
static bool frame_is(const uint8_t *buf, int len, uint8_t type, int min_len)
{
    return len >= min_len && buf[0] == UWB_MAGIC && buf[1] == UWB_VER && buf[2] == type;
}

// ===================== Стадии запуска =====================

// [1] Селфтест DWM1000. true = модуль исправен.
static bool stage_selftest(void)
{
    if (dwm1000_init() != ESP_OK) return false;
    ESP_LOGI(TAG, "[1/4] SELFTEST: DWM1000 OK (DEV_ID 0xDECA0130)");
    return true;
}

// [2] Рукопожатие: шлём PING, ждём PONG. Блокирует до успеха (с повтором).
static void stage_handshake(void)
{
    uwb_ping_t ping = { .hdr = { UWB_MAGIC, UWB_VER, UWB_FT_PING }, .node_id = NODE_ID_AIR };
    uint8_t rx[64];
    for (uint32_t tries = 1; ; tries++) {
        dwm1000_send_immediate((uint8_t *)&ping, sizeof(ping));
        int n = dwm1000_recv(rx, sizeof(rx), HS_REPLY_TIMEOUT_MS);
        if (frame_is(rx, n, UWB_FT_PONG, sizeof(uwb_ping_t))) {
            ESP_LOGI(TAG, "[2/4] HANDSHAKE: канал рабочий, земля ответила PONG (node %u)", rx[3]);
            return;
        }
        if (tries % 20 == 0)
            ESP_LOGW(TAG, "[2/4] HANDSHAKE: нет ответа земли (%" PRIu32 " попыток)...", tries);
        vTaskDelay(pdMS_TO_TICKS(HS_RETRY_PERIOD_MS));
    }
}

// [3] CONFIG + УДЕРЖАНИЕ (стадия 4/RUN — GATED). Шлём CONFIG, ждём CONFIG_ACK(OK); затем продолжаем
// слать CONFIG как keepalive (детектор живости земли). Возврат означает РЕСИНК — наверху повторяем
// рукопожатие:
//   * PONG в ответ на CONFIG  -> земля перезапустилась (рукопожатия в новой сессии не было);
//   * молчание HOLD_MAX_MISSES keepalive подряд -> земля недоступна (перезагрузка/потеря канала).
static void stage_config_and_hold(void)
{
    uwb_config_t cfg = {
        .hdr            = { UWB_MAGIC, UWB_VER, UWB_FT_CONFIG },
        .node_id        = NODE_ID_AIR,
        .carrier_hz     = CARRIER_HZ,
        .packet_rate_hz = PACKET_RATE_HZ,
        .pulses_per_pkt = PULSES_PER_PKT,
        .pulse_us       = PULSE_US_NOM,
        .gap_us         = GAP_US_NOM,
        .dt_us          = DT_US,
    };
    uint8_t rx[64];
    bool held = false;          // получили хотя бы один CONFIG_ACK(OK)
    uint32_t misses = 0;        // подряд идущих молчаний в удержании

    for (;;) {
        dwm1000_send_immediate((uint8_t *)&cfg, sizeof(cfg));
        int n = dwm1000_recv(rx, sizeof(rx), HS_REPLY_TIMEOUT_MS);

        if (frame_is(rx, n, UWB_FT_CONFIG_ACK, sizeof(uwb_ack_t))) {
            uint8_t status = rx[4];
            if (status == UWB_OK) {
                misses = 0;
                if (!held) {
                    held = true;
                    ESP_LOGI(TAG, "[3/4] CONFIG принят землёй; удержание — стадия 4/RUN GATED "
                                  "(restart-safe тест: перезагрузи любой узел, линк должен сам ресинкнуться)");
                }
                vTaskDelay(pdMS_TO_TICKS(HOLD_KEEPALIVE_MS));
                continue;
            }
            ESP_LOGW(TAG, "[3/4] CONFIG: земля отвергла конфиг (status=%u), повтор", status);
            vTaskDelay(pdMS_TO_TICKS(HS_RETRY_PERIOD_MS));
            continue;
        }

        if (frame_is(rx, n, UWB_FT_PONG, sizeof(uwb_ping_t))) {
            ESP_LOGW(TAG, "РЕСИНК: земля ответила PONG на CONFIG -> она ПЕРЕЗАПУСТИЛАСЬ; "
                          "возврат к рукопожатию");
            return;
        }

        // нет валидного ответа
        if (++misses >= HOLD_MAX_MISSES) {
            ESP_LOGW(TAG, "РЕСИНК: земля молчит %" PRIu32 " keepalive подряд -> возврат к рукопожатию",
                     misses);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(HS_RETRY_PERIOD_MS));
    }
}

// ===================== Стадия 5: рабочий цикл (код готов, запуск — после READY стадии 4) =====================

// Один выстрел рабочего цикла: двухимпульсная УЗ-пара. DATA = seq + паддинг-«мусор» до
// DATA_PAYLOAD_LEN (длина кадра задаёт длительность импульса, см. config.h).
static void run_cycle(uint32_t seq)
{
    uint8_t frame[DATA_PAYLOAD_LEN];
    uwb_data_t *d = (uwb_data_t *)frame;
    d->hdr       = (uwb_hdr_t){ UWB_MAGIC, UWB_VER, UWB_FT_DATA };
    d->pulse_idx = 1;
    d->seq       = seq;
    // хвост frame[8..] — паддинг: содержимое не важно («мусор» для выдерживания длительности)

    if (!dwm1000_send_immediate(frame, sizeof(frame))) return;   // импульс 1 (immediate)

#if ENABLE_ULTRASOUND && (PULSES_PER_PKT >= 2)
    uint64_t t1 = dwm1000_read_txstamp();
    d->pulse_idx = 2;
    dwm1000_send_delayed(frame, sizeof(frame), t1 + DT_TICKS);   // импульс 2 (delayed +ΔT)
#endif
}

__attribute__((unused))   // стадия 5 — запускается после READY; пока app_main её не вызывает
static void run_task(void *arg)
{
#if ENABLE_ULTRASOUND
    ESP_ERROR_CHECK(carrier_init());     // непрерывная несущая 25 кГц (только в рабочем цикле)
#endif
    uint32_t seq = 0;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        run_cycle(seq++);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PACKET_PERIOD_MS));
    }
}

// ===================== Фатал / main =====================

static void fault_loop(const char *why)
{
    for (;;) {
        ESP_LOGE(TAG, "ФАТАЛ: %s — работа остановлена, проверьте железо/перезагрузите", why);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "air_node старт: несущая %u Гц, пакеты %u Гц, имп~%u/зазор~%u мкс (DT %u)",
             (unsigned)CARRIER_HZ, (unsigned)PACKET_RATE_HZ,
             (unsigned)PULSE_US_NOM, (unsigned)GAP_US_NOM, (unsigned)DT_US);

    if (!stage_selftest()) fault_loop("селфтест DWM1000 не пройден");

    // Ресинк-цикл (restart-safe): рукопожатие -> CONFIG -> удержание с keepalive. При перезапуске
    // партнёра или потере канала stage_config_and_hold() возвращается, и мы заходим заново с [2].
    // !!! ГЕЙТ: стадия 4 (RUN: бутстрап первой позиции + run_task) пока НЕ запускается — проверяем,
    // что линк на стадии 3 самовосстанавливается после перезагрузки любого узла.
    for (;;) {
        stage_handshake();         // блокирует до PONG
        stage_config_and_hold();   // CONFIG + удержание; возврат = ресинк
    }
}
