//
// Передатчик DWM1000 на decadriver. Двухимпульсный пакет = ДВЕ UWB-передачи на цикл (вариант А):
// импульс 1 — dwm1000_send_immediate, импульс 2 — dwm1000_send_delayed через ΔT (DT_TICKS) от
// метки времени первой (dwm1000_read_txstamp). Несущая 25 кГц крутится непрерывно (carrier.c);
// каждый кадр поднимает EXTTXE, который гейтирует несущую на пьезо ровно на время кадра.
// fine-grain TX-seq ВЫКЛючен (dwt_setfinegraintxseq(0)) -> EXTTXE = один чистый строб, а не гребёнка.
//
#include "dwm1000.h"
#include "deca_glue.h"
#include "config.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include <inttypes.h>

static const char *TAG = "dwm1000";

// Должна совпадать с приёмником (src/rx_test/main/dwm1000_rx.c).
static dwt_config_t s_cfg = {
    .chan           = 5,
    .prf            = DWT_PRF_16M,
    .txPreambLength = DWT_PLEN_2048,    // штатная преамбула для 110k (надёжнее захват); PAC64 матчится с 2048
    .rxPAC          = DWT_PAC64,
    .txCode         = 4,
    .rxCode         = 4,
    .nsSFD          = 0,                // стандартный IEEE SFD (на 110k = 64 симв); ДОЛЖЕН совпадать с RX
    .dataRate       = DWT_BR_110K,      // 110 кбит/с: ~+15 дБ чувствительности (надёжный линк на слабых антеннах)
    .phrMode        = DWT_PHRMODE_STD,
    .sfdTO          = 2113,             // preamble(2048) + 1 + SFD(64) = 2113
};

// Селфтест + конфигурация DWM1000. Возвращает ESP_OK только если SPI поднялся, dwt_initialise
// прошёл и DEV_ID == 0xDECA0130 (иначе провал селфтеста — звать наверх как фатал).
esp_err_t dwm1000_init(void)
{
    esp_err_t e = deca_spi_init();
    if (e != ESP_OK) { ESP_LOGE(TAG, "SPI init: %s", esp_err_to_name(e)); return e; }

#if DEBUG_DWM
    deca_spi_diag();   // сырое чтение DEV_ID + round-trip записи — диагностика SPI-линка
#endif

    // ХОЛОДНЫЙ СТАРТ: DWM1000 может быть ещё не готов с первого раза (питание/страппинг GPIO5,6,
    // особенно с нагруженной линией EXTTXE). Повторяем сброс RSTn + dwt_initialise, пока не получим
    // корректный DEV_ID. Без этого узел «оживал» только после ручного RST.
    int attempt;
    for (attempt = 1; attempt <= 20; attempt++) {
        if (dwt_readdevid() == DWT_DEVICE_ID && dwt_initialise(DWT_LOADUCODE) != DWT_ERROR) break;
        deca_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (attempt > 20) {
        ESP_LOGE(TAG, "DWM1000 не отвечает (DEV_ID=0x%08" PRIX32 ") — SPI/питание/RSTn/страппинг",
                 (uint32_t)dwt_readdevid());
        return ESP_FAIL;
    }
    if (attempt > 1) ESP_LOGW(TAG, "DWM1000 готов с попытки %d (холодный старт)", attempt);

    dwt_configure(&s_cfg);
    // КРИТИЧНО: dwt_configure НЕ задаёт PG_DELAY и мощность TX — только dwt_configuretxrf. Без него
    // спектр TX искажён (приёмник ловит преамбулу, но не декодирует PHR). Значения для канала 5.
    dwt_txconfig_t txrf = { .PGdly = 0xC0, .power = 0x25456585 };
    dwt_configuretxrf(&txrf);
#if ENABLE_ULTRASOUND
    dwt_setlnapamode(0, 1);   // pa=1 -> GPIO5=EXTTXE (гейтирует ключи HC4066)
    dwt_setfinegraintxseq(0); // EXTTXE = один чистый строб на весь кадр (иначе гребёнка ~167 кГц)
#endif
    dwt_setrxtimeout(0);                              // приём (рукопожатие) без аппаратного таймаута
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFFUL);   // очистить липкие флаги статуса
    return ESP_OK;
}

static bool wait_txfrs(uint32_t timeout_us)
{
    for (uint32_t t = 0; t < timeout_us; t += 20) {
        if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);   // сброс флага
            return true;
        }
        esp_rom_delay_us(20);
    }
    ESP_LOGW(TAG, "TXFRS timeout");
    return false;
}

bool dwm1000_send_immediate(const uint8_t *data, size_t len)
{
    dwt_writetxdata(len + 2, (uint8_t *)data, 0);   // +2: CRC генерится аппаратно
    dwt_writetxfctrl(len + 2, 0, 0);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) return false;
    return wait_txfrs(PULSE_US_NOM + 3000);
}

bool dwm1000_send_delayed(const uint8_t *data, size_t len, uint64_t dx_time)
{
    dwt_writetxdata(len + 2, (uint8_t *)data, 0);
    dwt_writetxfctrl(len + 2, 0, 0);
    // dwt_setdelayedtrxtime пишет старшие 32 бита (биты 39:8); младшие 9 игнорируются HW
    dwt_setdelayedtrxtime((uint32_t)(dx_time >> 8));
    if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
        ESP_LOGW(TAG, "delayed TX too late (DX_TIME в прошлом)");
        return false;
    }
    return wait_txfrs(DT_US + 3000);
}

uint64_t dwm1000_read_txstamp(void)
{
    uint8_t ts[5];
    dwt_readtxtimestamp(ts);
    return (uint64_t)ts[0] | ((uint64_t)ts[1] << 8) | ((uint64_t)ts[2] << 16)
         | ((uint64_t)ts[3] << 24) | ((uint64_t)ts[4] << 32);
}

// Приём одного кадра с программным таймаутом (для рукопожатия — слушать ответ земли).
// Возвращает длину полезной нагрузки (без 2 CRC), либо 0 при таймауте/ошибке приёма.
int dwm1000_recv(uint8_t *buf, size_t bufsize, uint32_t timeout_ms)
{
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR);  // сброс старых флагов
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    for (uint32_t waited = 0; ; waited += portTICK_PERIOD_MS) {
        uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
        if (status & SYS_STATUS_RXFCG) {
            uint16_t len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            uint16_t pl  = (len >= 2) ? (uint16_t)(len - 2) : 0;   // без CRC
            if (pl > bufsize) pl = (uint16_t)bufsize;
            if (pl) dwt_readrxdata(buf, pl, 0);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            return (int)pl;
        }
        if (status & SYS_STATUS_ALL_RX_ERR) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            return 0;
        }
        if (waited >= timeout_ms) { dwt_forcetrxoff(); return 0; }
        vTaskDelay(1);
    }
}
