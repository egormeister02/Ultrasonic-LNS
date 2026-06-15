#pragma once
//
// Общий протокол UWB-линка системы УЗ-навигации: air_node (передатчик/мастер) <-> ground_node
// (приёмник/центр). Подключается ОБОИМИ проектами через общий компонент src/components/uwb_protocol.
//
// DW1000 — половинно-дуплексный: мастер (air_node) шлёт запрос и слушает ответ, ground отвечает.
// Запуск идёт СТРОГО по стадиям, без работы вслепую:
//
//   1. SELFTEST  — каждый узел проверяет свой DWM1000 (DEV_ID + dwt_initialise).
//   2. HANDSHAKE — air шлёт PING, ground отвечает PONG -> UWB-канал в принципе рабочий.
//   3. CONFIG    — air шлёт CONFIG (свой тайминг), ground принимает, подстраивается и шлёт
//                  CONFIG_ACK. Дальше air удерживает связь keepalive-CONFIG'ами.
//   4. RUN       — рабочий цикл: air шлёт DATA (номер пакета + паддинг) синхронно с УЗ-парой,
//                  ground отвечает квитанцией (стадия/маска якорей/валидность фикса).
//
// RESTART-SAFE (ресинк по стадиям, без epoch): стадия партнёра кодируется ТИПОМ его ответного кадра
// (PONG=рукопожатие, CONFIG_ACK=конфиг, квитанция=рабочий цикл) — отдельного поля нет. Узел, который
// видит ответ более РАННЕЙ стадии (партнёр перезагрузился) либо тишину N запросов подряд, опускается
// к рукопожатию. PONG служит и ответом на PING, и сигналом регрессии: ground отвечает PONG на
// CONFIG/DATA, если в текущей сессии ещё не прошёл рукопожатие. Инициатор подъёма — всегда air.
//
// Все кадры начинаются общей шапкой {magic, ver, type}. CRC (2 байта) добавляет аппаратно DW1000 —
// в структурах его НЕТ. Структуры __packed: реальная компоновка байт в эфире.
//
#include <stdint.h>

#define UWB_MAGIC   0x55u   // 'U' — признак кадра нашей системы (фильтр чужого/шумового трафика)
#define UWB_VER     1u      // версия протокола; несовместимое изменение раскладки -> увеличить

// Типы кадров (поле type общей шапки)
enum {
    UWB_FT_PING       = 0x01,  // air  -> ground: проверка канала (рукопожатие)
    UWB_FT_PONG       = 0x02,  // ground -> air : «жив», ответ на PING
    UWB_FT_CONFIG     = 0x03,  // air  -> ground: параметры тайминга передатчика
    UWB_FT_CONFIG_ACK = 0x04,  // ground -> air : «конфиг принят» (стадия 3)
    UWB_FT_READY      = 0x05,  // ground -> air : квитанция рабочего цикла / «бутстрап-лок» (стадия 4)
    UWB_FT_DATA       = 0x06,  // air  -> ground: рабочий цикл (номер пакета + паддинг) (стадия 4)
};

// Коды результата для подтверждений (поле status)
enum {
    UWB_OK       = 0x00,  // всё в порядке
    UWB_ERR_BUSY = 0x01,  // занят/не готов
    UWB_ERR_CFG  = 0x02,  // конфиг отвергнут (версия/значения)
};

// Общая шапка — первые 3 байта ЛЮБОГО кадра
typedef struct __attribute__((packed)) {
    uint8_t magic;   // == UWB_MAGIC
    uint8_t ver;     // == UWB_VER
    uint8_t type;    // UWB_FT_*
} uwb_hdr_t;

// PING / PONG — рукопожатие. Кроме шапки несёт только id узла.
typedef struct __attribute__((packed)) {
    uwb_hdr_t hdr;        // type = UWB_FT_PING / UWB_FT_PONG
    uint8_t   node_id;
} uwb_ping_t;             // 4 байта

// CONFIG — передатчик объявляет тайминг; приёмник подстраивает окна детекта (всегда С ЗАПАСОМ).
typedef struct __attribute__((packed)) {
    uwb_hdr_t hdr;            // type = UWB_FT_CONFIG
    uint8_t   node_id;
    uint16_t  carrier_hz;     // несущая, Гц (напр. 25000)
    uint8_t   packet_rate_hz; // пакетов/с (пар импульсов)
    uint8_t   pulses_per_pkt; // импульсов в паре (обычно 2)
    uint16_t  pulse_us;       // НОМИНАЛ длительности импульса (окно EXTTXE), мкс
    uint16_t  gap_us;         // НОМИНАЛ зазора между импульсами, мкс
    uint16_t  dt_us;          // разнос СТАРТОВ импульсов (= pulse_us + gap_us), мкс
} uwb_config_t;               // 14 байт

// CONFIG_ACK / READY — подтверждение стадии землёй.
typedef struct __attribute__((packed)) {
    uwb_hdr_t hdr;        // type = UWB_FT_CONFIG_ACK / UWB_FT_READY
    uint8_t   node_id;
    uint8_t   status;     // UWB_OK или код проблемы
} uwb_ack_t;             // 5 байт

// DATA — рабочий цикл (стадия 5). Полезного пока нет: номер пакета + паддинг-«мусор» для
// выдерживания нужной длительности импульса (длину задаёт DATA_PAYLOAD_LEN в air_node/config.h).
typedef struct __attribute__((packed)) {
    uwb_hdr_t hdr;        // type = UWB_FT_DATA
    uint8_t   pulse_idx;  // 1 или 2 — номер импульса в паре (для парности δ-калибровки на земле)
    uint32_t  seq;        // номер пакета (цикла)
    // далее в эфире идёт паддинг произвольными байтами до DATA_PAYLOAD_LEN (см. air_node/config.h)
} uwb_data_t;             // 8 байт «полезной» шапки данных, далее паддинг
