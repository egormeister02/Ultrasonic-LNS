//
// Приём UWB-кадров через DWM1000 на decadriver. Непрерывный приём опросом
// SYS_STATUS (без прерываний): включаем RX, ждём RXFCG (кадр принят) или ошибку,
// читаем полезную нагрузку и метку времени приёма, печатаем, повторяем.
//
#include "dwm1000_rx.h"
#include "deca_glue.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <inttypes.h>
#include <stdbool.h>
#include <math.h>

static const char *TAG = "dwm_rx";

// FREQ_OFFSET_MULTIPLIER_110KB и HERTZ_TO_PPM_MULTIPLIER_CHAN_5 определены в deca_device_api.h
// (формула Decawave, ch5, 110 кбит/с). carrier integrator * эти константы = ppm сдвига частоты TX/RX.
// Большой |ppm| (>~20) при исправной преамбуле = расхождение кварцев -> лечится dwt_setxtaltrim.

// Должна совпадать с конфигурацией передатчика (src/proto: ch5/PRF16/PLEN1024/6.8M).
static dwt_config_t s_cfg = {
    .chan           = 5,
    .prf            = DWT_PRF_16M,
    .txPreambLength = DWT_PLEN_2048,    // штатная преамбула для 110k (надёжнее захват); PAC64 матчится с 2048
    .rxPAC          = DWT_PAC64,
    .txCode         = 4,
    .rxCode         = 4,
    .nsSFD          = 0,                // стандартный IEEE SFD (на 110k = 64 симв); ДОЛЖЕН совпадать с TX
    .dataRate       = DWT_BR_110K,      // 110 кбит/с: ~+15 дБ чувствительности
    .phrMode        = DWT_PHRMODE_STD,
    .sfdTO          = 2113,             // preamble(2048) + 1 + SFD(64) = 2113
};

esp_err_t dwm1000_rx_init(void)
{
    esp_err_t e = deca_spi_init();
    if (e != ESP_OK) { ESP_LOGE(TAG, "spi init: %s", esp_err_to_name(e)); return e; }

    deca_spi_diag();   // сырое чтение DEV_ID до dwt_initialise — диагностика SPI-линка

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        ESP_LOGE(TAG, "dwt_initialise failed — проверь SPI/питание/RSTn");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "DEV_ID = 0x%08" PRIX32 " (ожидается 0x%08X)",
             (uint32_t)dwt_readdevid(), (unsigned)DWT_DEVICE_ID);

    dwt_configure(&s_cfg);
    dwt_setrxtimeout(0);   // приём без таймаута
    // очистить ВСЕ липкие флаги статуса (в т.ч. CLKPLL_LL bit25, SLP2INIT bit23). Если в
    // статусах приёма CLKPLL_LL (0x02000000) снова появляется — тактовый PLL реально теряет
    // лок (питание/развязка модуля), а не остаточный флаг.
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFFUL);
    ESP_LOGI(TAG, "RX configured: ch5, PRF16, PLEN2048, 110k, pcode4");
    return ESP_OK;
}

static void rx_task(void *arg)
{
    uint8_t rx[127];
    uint32_t got = 0;

    // Автоподстройка кварца к нулю частотного сдвига (hill-climb по off_ppm).
    uint8_t trim = dwt_getinitxtaltrim();   // старт от заводского/mid (0x10)
    int8_t  dir  = -1;                       // направление пробного шага тримма
    double  prev_abs = 1e9;
    bool    tuning = true;
    dwt_setxtaltrim(trim);

    for (;;) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status;
        for (;;) {
            status = dwt_read32bitreg(SYS_STATUS_ID);
            if (status & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)) break;
            vTaskDelay(1);   // >=1 тик: pdMS_TO_TICKS(2) при тике 100 Гц округляется в 0 -> голодает IDLE1 -> task_wdt
        }

        // сдвиг несущей TX/RX (преамбула/SFD уже обработаны и на ошибочном кадре)
        double off_ppm = (double)dwt_readcarrierintegrator()
                       * FREQ_OFFSET_MULTIPLIER_110KB * HERTZ_TO_PPM_MULTIPLIER_CHAN_5;

        // уровень сигнала/шум (формулы DW1000 user manual §4.7, A=113.77 для PRF 16 МГц)
        dwt_rxdiag_t d;
        dwt_readdiagnostics(&d);
        double N    = d.rxPreamCount ? (double)d.rxPreamCount : 1.0;
        double C    = d.maxGrowthCIR ? (double)d.maxGrowthCIR : 1.0;
        double fpsq = (double)d.firstPathAmp1 * d.firstPathAmp1
                    + (double)d.firstPathAmp2 * d.firstPathAmp2
                    + (double)d.firstPathAmp3 * d.firstPathAmp3;
        if (fpsq < 1.0) fpsq = 1.0;
        double rx_dbm = 10.0 * log10(C * 131072.0 / (N * N)) - 113.77;   // мощность принятого
        double fp_dbm = 10.0 * log10(fpsq / (N * N)) - 113.77;           // мощность первого пути

        // тянем |off| к нулю триммом кварца, разворачивая шаг при ухудшении.
        // ВАЖНО: подстраиваем ТОЛЬКО когда SFD реально пойман (RXSFDD) — иначе на событиях
        // RXSFDTO (preamc=0) carrier integrator невалиден (off≈мусор ~1ppm) и servo сходится не туда.
        if (tuning && (status & SYS_STATUS_RXSFDD)) {
            double a = fabs(off_ppm);
            if (a <= 3.0) {
                tuning = false;
                ESP_LOGW(TAG, "XTAL trim ГОТОВ: trim=%u off=%.1f ppm", trim, off_ppm);
            } else {
                if (a > prev_abs) dir = (int8_t)(-dir);    // стало хуже -> развернуть
                prev_abs = a;
                int nt = (int)trim + dir;
                if (nt < 0)  { nt = 0;  dir = +1; }
                if (nt > 31) { nt = 31; dir = -1; }
                trim = (uint8_t)nt;
                dwt_setxtaltrim(trim);
                ESP_LOGW(TAG, "XTAL tune: off=%.1f ppm -> trim=%u (dir=%+d)", off_ppm, trim, dir);
            }
        }

        if (status & SYS_STATUS_RXFCG) {
            uint16_t len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (len > sizeof(rx)) len = sizeof(rx);
            dwt_readrxdata(rx, len, 0);

            uint8_t ts[5];
            dwt_readrxtimestamp(ts);
            uint64_t rx_ts = (uint64_t)ts[0] | ((uint64_t)ts[1] << 8) | ((uint64_t)ts[2] << 16)
                           | ((uint64_t)ts[3] << 24) | ((uint64_t)ts[4] << 32);

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);   // сброс флага

            uint16_t pl = (len >= 2) ? (uint16_t)(len - 2) : 0;   // полезная нагрузка без FCS
            ESP_LOGI(TAG, "#%" PRIu32 " RX st=0x%08" PRIX32 " len=%u off=%.1f ppm rx_ts=%" PRIu64 "  data: %02X %02X %02X %02X %02X",
                     ++got, status, pl, off_ppm, rx_ts,
                     (pl > 0) ? rx[0] : 0, (pl > 1) ? rx[1] : 0, (pl > 2) ? rx[2] : 0,
                     (pl > 3) ? rx[3] : 0, (pl > 4) ? rx[4] : 0);
        } else {
            // ошибка приёма: залогировать статус и сдвиг, сбросить флаги, ресетнуть приёмник
            ESP_LOGW(TAG, "RX err st=0x%08" PRIX32 " off=%.1f ppm  rx=%.0f fp=%.0f dBm noise=%u preamc=%u",
                     status, off_ppm, rx_dbm, fp_dbm, d.stdNoise, d.rxPreamCount);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
        }
    }
}

void dwm1000_rx_start(void)
{
    xTaskCreatePinnedToCore(rx_task, "dwm_rx", 4096, NULL, 9, NULL, 1);
}
