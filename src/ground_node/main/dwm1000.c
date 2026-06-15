//
// DWM1000 узла ground_node: приём кадров от air_node и передача ответов (PONG / CONFIG_ACK).
// Непрерывный приём опросом SYS_STATUS (без прерываний). Попутно сводит XTAL-trim к нулю
// частотного сдвига несущей (servo по off_ppm) — это давало 100% приём на тестовом стенде.
//
#include "dwm1000.h"
#include "config.h"
#include "deca_glue.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include <inttypes.h>
#include <math.h>

static const char *TAG = "dwm1000";

// ДОЛЖНА совпадать с air_node (src/air_node/main/dwm1000.c).
static dwt_config_t s_cfg = {
    .chan           = 5,
    .prf            = DWT_PRF_16M,
    .txPreambLength = DWT_PLEN_2048,
    .rxPAC          = DWT_PAC64,        // матчится с PLEN2048
    .txCode         = 4,
    .rxCode         = 4,
    .nsSFD          = 0,                // стандартный IEEE SFD (на 110k = 64 симв)
    .dataRate       = DWT_BR_110K,
    .phrMode        = DWT_PHRMODE_STD,
    .sfdTO          = 2113,             // preamble(2048) + 1 + SFD(64)
};

// Состояние авто-подстройки кварца (hill-climb по |off_ppm|).
static uint8_t s_trim;
static int8_t  s_dir     = -1;
static double  s_prevabs = 1e9;
static bool    s_tuning  = true;

esp_err_t dwm1000_init(void)
{
    esp_err_t e = deca_spi_init();
    if (e != ESP_OK) { ESP_LOGE(TAG, "SPI init: %s", esp_err_to_name(e)); return e; }

#if DEBUG_DWM
    deca_spi_diag();
#endif

    // ХОЛОДНЫЙ СТАРТ: повторяем сброс RSTn + dwt_initialise, пока не получим корректный DEV_ID
    // (модуль может быть не готов с первого раза). Без ретрая узел оживал только после ручного RST.
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
    // TXRF нужен и приёмнику — он передаёт ответы (PONG/ACK); без него спектр TX искажён.
    dwt_txconfig_t txrf = { .PGdly = 0xC0, .power = 0x25456585 };
    dwt_configuretxrf(&txrf);

    dwt_setrxtimeout(0);                              // приём без аппаратного таймаута кадра
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFFUL);   // очистить ВСЕ липкие флаги (вкл. CLKPLL_LL)

    s_trim = dwt_getinitxtaltrim();                   // старт от заводского/mid
    dwt_setxtaltrim(s_trim);
    return ESP_OK;
}

// Подвести XTAL-trim к нулю частотного сдвига. Валиден только когда реально пойман SFD (RXSFDD),
// иначе carrier integrator = мусор. Останавливается при |off| <= 3 ppm.
static void xtal_tune(uint32_t status)
{
    if (!s_tuning || !(status & SYS_STATUS_RXSFDD)) return;
    double off_ppm = (double)dwt_readcarrierintegrator()
                   * FREQ_OFFSET_MULTIPLIER_110KB * HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
    double a = fabs(off_ppm);
    if (a <= 3.0) {
        s_tuning = false;
        ESP_LOGI(TAG, "XTAL trim сведён: trim=%u off=%.1f ppm", s_trim, off_ppm);
        return;
    }
    if (a > s_prevabs) s_dir = (int8_t)(-s_dir);   // стало хуже -> развернуть шаг
    s_prevabs = a;
    int nt = (int)s_trim + s_dir;
    if (nt < 0)  { nt = 0;  s_dir = +1; }
    if (nt > 31) { nt = 31; s_dir = -1; }
    s_trim = (uint8_t)nt;
    dwt_setxtaltrim(s_trim);
}

static uint32_t s_rx_errors;
static uint32_t s_last_err;
static bool     s_rx_armed;

uint32_t dwm1000_rx_errors(void)   { return s_rx_errors; }
uint32_t dwm1000_last_rx_err(void) { return s_last_err; }

// Приём как на проверенном стенде rx_test: приёмник держим АРМИРОВАННЫМ непрерывно (без
// forcetrxoff между квантами ожидания); пере-армируем только после кадра/ошибки/передачи.
int dwm1000_recv(uint8_t *buf, size_t bufsize, uint32_t timeout_ms)
{
    if (!s_rx_armed) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        s_rx_armed = true;
    }

    for (uint32_t waited = 0; ; waited += portTICK_PERIOD_MS) {
        uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

        if (status & SYS_STATUS_RXFCG) {
            xtal_tune(status);
            uint16_t len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            uint16_t pl  = (len >= 2) ? (uint16_t)(len - 2) : 0;   // без CRC
            if (pl > bufsize) pl = (uint16_t)bufsize;
            if (pl) dwt_readrxdata(buf, pl, 0);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            s_rx_armed = false;                 // кадр снят -> пере-армировать на след. вызове
            return (int)pl;
        }
        if (status & SYS_STATUS_ALL_RX_ERR) {
            xtal_tune(status);                  // SFD мог быть пойман даже на ошибочном кадре
            s_rx_errors++;
            s_last_err = status;                // тип ошибки (биты SYS_STATUS) — для диагностики
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            s_rx_armed = false;
            return 0;
        }
        if (waited >= timeout_ms) return 0;     // кадра нет; приёмник ОСТАЁТСЯ армированным
        vTaskDelay(1);
    }
}

bool dwm1000_send(const uint8_t *data, size_t len)
{
    dwt_forcetrxoff();                               // выйти из RX перед передачей
    s_rx_armed = false;                              // приёмник снят -> recv пере-армирует после
    dwt_writetxdata(len + 2, (uint8_t *)data, 0);    // +2: CRC генерится аппаратно
    dwt_writetxfctrl(len + 2, 0, 0);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) return false;
    for (uint32_t t = 0; t < 8000; t += 20) {        // ждём TXFRS (макс ~8 мс на кадр)
        if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
            return true;
        }
        esp_rom_delay_us(20);
    }
    ESP_LOGW(TAG, "TXFRS timeout");
    return false;
}
