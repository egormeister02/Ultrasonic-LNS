#pragma once
//
// Карта пинов ESP32-S3 DevKitC-1 <-> DWM1000 / тракт излучения (узел БПЛА).
// SPI-периферия: FSPI (SPI2_HOST). Номера в комментариях — физические контакты
// модуля DWM1000 (полная разводка — pinout_uav.md).
//

// --- Выходы несущей -> входы ключей SN74HC4066 ---
#define CARRIER_A_PIN   4   // несущая 25 кГц, фаза 0°   -> HC4066 1A -> TC4427A INA -> пьезо(+)
#define CARRIER_B_PIN   5   // несущая 25 кГц, фаза 180° -> HC4066 2A -> TC4427A INB -> пьезо(−)

// --- SPI-шина DWM1000 (FSPI) ---
#define DWM_SCK_PIN     12  // -> DWM1000 SPICLK  (pad 20)
#define DWM_MOSI_PIN    11  // -> DWM1000 SPIMOSI (pad 18)
#define DWM_MISO_PIN    13  // <- DWM1000 SPIMISO (pad 19)
#define DWM_CS_PIN      10  // -> DWM1000 SPICSn  (pad 17), внешний pull-up 10к на 3V3

// --- Управление / статус DWM1000 ---
#define DWM_IRQ_PIN     14  // <- DWM1000 IRQ  (pad 22), активный высокий
#define DWM_RST_PIN     7   // -> DWM1000 RSTn (pad 3), open-drain, внешний pull-up 10к

// ПРИМЕЧАНИЕ: DWM1000 GPIO5/EXTTXE (pad 10) управляет разрешением ключей SN74HC4066
// аппаратно — это НЕ пин MCU. В v1 он управляется как программный GPIO выхода через
// SPI-регистры DWM1000 (PMSC clock + GPIO_MODE/DIR/DOUT). Strapping-контакты GPIO5
// (pad 10) и GPIO6 (pad 9) требуют внешних pull-down 10к на GND, иначе модуль на бутe
// выберет неверный SPI mode.
