//
// Платформенный слой decadriver для ESP32-S3 — АППАРАТНЫЙ SPI (esp_driver_spi), режим 0, ручной CS.
//
// Bring-up DW1000 на ESP32-S3 потребовал закрыть ТРИ независимых дефекта (каждый со своим симптомом);
// решение свёрстано по рабочему порту github.com/wjxway/DW1000_ESP32:
//   1) DMA на мелком rx-буфере -> молчаливый сбой/нули. Фикс: SPI_DMA_DISABLED (FIFO; транзакции <64 Б).
//   2) Аппаратный CS (spics_io_num) в full-duplex на S3 -> слейв НЕ выбирается, MISO=0 при ESP_OK
//      (давало DEV_ID=0x00000000). Фикс: CS ВРУЧНУЮ (spics_io_num=-1, дёргаем GPIO сами вокруг
//      транзакции). Плюс ПРАЙМ-чтение в init: режим SPI применяется в начале 1-й транзакции, позже
//      опускания CS, поэтому самое первое чтение невалидно — делаем холостое. ОДИН device-handle
//      (смена хэндла = снова «первая невалидная транзакция»).
//   3) Этот модуль/разводка отдаёт поток MISO на 1 такт РАНЬШЕ (только чтение; MOSI/запись нативно
//      верны — round-trip PANADR: записали 21 43 65 87, чип хранит верно, а сырое чтение даёт <<1).
//      Фикс: де-сдвиг ТОЛЬКО чтения (DWM_MISO_DESHIFT) — читаем rx-поток со смещением -1 бит. Держим
//      ЕДИНЫЙ клок (SPI_CLK_HZ), без переключения на быстрый: выше ~6 МГц сдвиг, по сообщениям, меняется.
//
// Пины 10..13 = IO_MUX FSPI на ESP32-S3 (CS10/MOSI11/CLK12/MISO13) -> быстрый путь без GPIO-матрицы.
// Доступ к SPI однопоточный (один RX/TX-таск) -> общий статический scratch-буфер и no-op мьютексы ОК.
//
#include "deca_glue.h"
#include "deca_device_api.h"   // типы uint8/uint16/uint32, decaIrqStatus_t, DWT_DEVICE_ID
#include "dwm1000_pins.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <inttypes.h>
#include <stdbool.h>

#define DWM_SPI_HOST        SPI2_HOST        // FSPI; пины 10..13 = IO_MUX FSPI на ESP32-S3
#define SPI_CLK_HZ          (2 * 1000 * 1000) // ЕДИНЫЙ клок. <=3 МГц обязателен в INIT (до лока CLKPLL);
                                              // держим единым и в работе, чтобы 1-битный сдвиг MISO был
                                              // постоянным (выше ~6 МГц он, по сообщениям, меняется ->
                                              // фикс. де-сдвиг сломался бы). 2 МГц с запасом хватает:
                                              // кадры ~7 Б, темп 10 Гц. ОДИН хэндл -> нет «первой
                                              // невалидной транзакции» при смене устройства.
#define DWM_MISO_DESHIFT    1                // этот модуль отдаёт MISO на 1 такт РАНЬШЕ (только чтение);
                                              // компенсируем ТОЛЬКО чтение, запись/MOSI нативно верны.
#define SPI_SCRATCH_MAX     64               // потолок FIFO без DMA; наши транзакции мелкие (DEV_ID 5Б, кадр ~7Б)

static spi_device_handle_t s_spi;    // единственный хэндл устройства (единый клок SPI_CLK_HZ)

// DMA-совместимые (внутренняя SRAM, выровнены по слову) статические буферы обмена.
WORD_ALIGNED_ATTR static uint8_t s_txbuf[SPI_SCRATCH_MAX];
WORD_ALIGNED_ATTR static uint8_t s_rxbuf[SPI_SCRATCH_MAX];

// Одна full-duplex транзакция в пределах одного CS: тактуем (header+body) байт. На запись body=wbuf
// (rbuf=NULL), на чтение body=нули и захватываем MISO в rbuf. decadriver ждёт header, затем тело.
static int spi_xfer(uint16 hlen, const uint8 *hbuf, uint32 blen, const uint8 *wbuf, uint8 *rbuf)
{
    uint32_t total = (uint32_t)hlen + blen;
    if (total == 0) return 0;
    if (total > SPI_SCRATCH_MAX) {
        ESP_LOGE("deca_glue", "SPI xfer %" PRIu32 " > scratch %d", total, SPI_SCRATCH_MAX);
        return -1;
    }

    for (uint16 i = 0; i < hlen; i++)  s_txbuf[i] = hbuf[i];
    for (uint32 i = 0; i < blen; i++)  s_txbuf[hlen + i] = wbuf ? wbuf[i] : 0x00;

    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = s_txbuf,
        .rx_buffer = rbuf ? s_rxbuf : NULL,
    };
    // CS опускаем САМИ на всю транзакцию (header+body в одном CS), драйвер его не трогает.
    gpio_set_level(DWM_CS_PIN, 0);
    esp_err_t e = spi_device_polling_transmit(s_spi, &t);
    gpio_set_level(DWM_CS_PIN, 1);
    if (e != ESP_OK) {
        ESP_LOGE("deca_glue", "spi_xfer(%" PRIu32 "B) FAIL: %s", total, esp_err_to_name(e));
        return -1;
    }

    if (rbuf) {
#if DWM_MISO_DESHIFT
        // MISO приходит на 1 такт раньше: пойманный бит на позиции i = реальный бит потока s[i+1].
        // Значит реальный бит данных j = бит (hlen*8 + j - 1) пойманного потока. Реконструируем,
        // читая rx-поток со смещением -1 бит (MSB-first внутри байта). MOSI/запись НЕ трогаем.
        uint32_t base = hlen * 8u - 1u;                    // hlen>=1 -> base>=7, без знакового переполнения
        for (uint32 k = 0; k < blen; k++) {
            uint8_t v = 0;
            for (uint32 b = 0; b < 8; b++) {
                uint32_t p = base + k * 8u + b;            // p_max = (hlen+blen)*8-2 < захваченных бит
                v = (uint8_t)((v << 1) | ((s_rxbuf[p >> 3] >> (7 - (p & 7))) & 1));
            }
            rbuf[k] = v;
        }
#else
        for (uint32 i = 0; i < blen; i++) rbuf[i] = s_rxbuf[hlen + i];
#endif
    }
    return 0;
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodyLength, const uint8 *bodyBuffer)
{
    return spi_xfer(headerLength, headerBuffer, bodyLength, bodyBuffer, NULL);
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readLength, uint8 *readBuffer)
{
    return spi_xfer(headerLength, headerBuffer, readLength, NULL, readBuffer);
}

// Приём опросом, прерывание DW1000 не используем -> мьютексы no-op.
decaIrqStatus_t decamutexon(void)       { return 0; }
void            decamutexoff(decaIrqStatus_t s) { (void)s; }

void deca_sleep(unsigned int time_ms)
{
    vTaskDelay(pdMS_TO_TICKS(time_ms ? time_ms : 1));
}

// Аппаратный сброс DWM1000 по RSTn + холостой прайм SPI. Вынесено отдельно, чтобы ПОВТОРЯТЬ сброс
// при инициализации: на холодном старте модуль (особенно с нагруженной линией EXTTXE) может быть
// не готов с первого раза — тогда вызывающий повторяет deca_reset() + dwt_initialise.
void deca_reset(void)
{
    // RSTn open-drain: тянем в 0, отпускаем в high-Z — гнать в high НЕЛЬЗЯ.
    gpio_set_direction(DWM_RST_PIN, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(DWM_RST_PIN, 0);
    esp_rom_delay_us(2000);
    gpio_set_direction(DWM_RST_PIN, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(5));

    // ПРАЙМ: первое чтение после применения SPI-режима невалидно — холостая транзакция.
    uint8_t ph = 0x00, pb[4];
    spi_xfer(1, &ph, 4, NULL, pb);
}

esp_err_t deca_spi_init(void)
{
    spi_bus_config_t bus = {
        .sclk_io_num     = DWM_SCK_PIN,
        .mosi_io_num     = DWM_MOSI_PIN,
        .miso_io_num     = DWM_MISO_PIN,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 0,                // 0 -> драйвер берёт потолок FIFO (DMA отключён)
    };
    // DMA отключён: все наши транзакции мелкие (<64 Б — потолок FIFO), а FIFO-режим снимает
    // требования к DMA-совместимости/выравниванию rx-буфера (типовая причина «читаются нули»).
    esp_err_t e = spi_bus_initialize(DWM_SPI_HOST, &bus, SPI_DMA_DISABLED);
    if (e != ESP_OK) return e;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = SPI_CLK_HZ,
        .mode           = 0,                 // CPOL=0, CPHA=0 — DW1000 страпнут на mode0 (GPIO5/6 на GND)
        .spics_io_num   = -1,                // CS ВРУЧНУЮ (см. ниже): на S3 аппаратный CS в full-duplex глючит
        .queue_size     = 1,
        .input_delay_ns = 0,                 // на 2 МГц марджин огромный
    };
    e = spi_bus_add_device(DWM_SPI_HOST, &dev, &s_spi);
    if (e != ESP_OK) return e;

    // CS — ручной GPIO-выход (spics_io_num=-1), идл = высокий. На ESP32-S3 аппаратный CS в
    // full-duplex не выбирает слейв корректно (MISO=0 при OK-транзакции); рабочий порт DW1000
    // под S3 (wjxway/DW1000_ESP32) тоже дёргает CS вручную.
    gpio_set_direction(DWM_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DWM_CS_PIN, 1);

    deca_reset();   // сброс RSTn + прайм SPI (повторяемый — см. ретрай в dwm1000_init)
    return ESP_OK;
}

// ---------- Диагностика ----------
// Чтение DEV_ID (с де-сдвигом MISO) и round-trip записи PANADR (MOSI нативно). Должны видеть
// DEV_ID=0xDECA0130 и обратное чтение 21 43 65 87 — тогда R/W корректны и dwt_initialise пройдёт.
void deca_spi_diag(void)
{
    static const char *D = "deca_diag";
    ESP_LOGI(D, "HW SPI mode0 @%d МГц, ручной CS: SCK=%d MOSI=%d MISO=%d CS=%d RST=%d | жду DEV_ID=0x%08X",
             SPI_CLK_HZ / 1000000, DWM_SCK_PIN, DWM_MOSI_PIN, DWM_MISO_PIN, DWM_CS_PIN, DWM_RST_PIN,
             (unsigned)DWT_DEVICE_ID);

    uint32_t id = deca_read_devid();
    bool rd_ok = (id == (uint32_t)DWT_DEVICE_ID);
    ESP_LOGW(D, "DEV_ID (read de-shift %s): 0x%08" PRIX32 "%s", DWM_MISO_DESHIFT ? "вкл" : "выкл", id,
             rd_ok ? "  <== OK, чтение корректно"
                   : (id == 0xBC950360UL ? "  <== сырой сдвиг <<1 (де-сдвиг не сошёлся?)"
                                         : "  (не совпало — проверь питание/RSTn/пины/страппинг)"));

    // round-trip записи: PANADR (0x03, R/W 4 байта) — пишем шаблон, читаем назад.
    static const uint8_t PAT[4] = { 0x21, 0x43, 0x65, 0x87 };
    uint8_t whdr = (uint8_t)(0x80 | 0x03);   // write, reg 0x03, без sub-index
    writetospi(1, &whdr, 4, PAT);
    uint8_t rhdr = 0x03, back[4] = { 0 };
    readfromspi(1, &rhdr, 4, back);
    bool wr_ok = (back[0] == PAT[0] && back[1] == PAT[1] && back[2] == PAT[2] && back[3] == PAT[3]);
    ESP_LOGW(D, "PANADR write->read: %02X %02X %02X %02X (ждал 21 43 65 87)%s",
             (unsigned)back[0], (unsigned)back[1], (unsigned)back[2], (unsigned)back[3],
             wr_ok ? "  <== ЗАПИСЬ OK" : "  (запись не подтвердилась)");

    if (rd_ok && wr_ok)
        ESP_LOGW(D, "ВЕРДИКТ: аппаратный SPI mode0 + ручной CS + де-сдвиг чтения -> R/W корректны. "
                    "dwt_initialise должен пройти.");
    else
        ESP_LOGE(D, "ВЕРДИКТ: остаточная проблема SPI. Если DEV_ID=0xBC950360 — де-сдвиг чтения не "
                    "сошёлся; иначе питание/RSTn/страппинг GPIO5,6.");
}

// Однократное чтение DEV_ID (рег. 0x00) — для диагностики и живого мониторинга линка.
uint32_t deca_read_devid(void)
{
    uint8_t hdr = 0x00, b[4] = {0};
    readfromspi(1, &hdr, 4, b);
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
