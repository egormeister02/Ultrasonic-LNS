//
// Непрерывная противофазная несущая 25 кГц через MCPWM-драйвер ESP-IDF.
//
// Один таймер -> один оператор -> один компаратор (50%) -> два генератора:
//   genA (CARRIER_A_PIN): HIGH при counter==0, LOW при compare  (фаза 0°)
//   genB (CARRIER_B_PIN): LOW  при counter==0, HIGH при compare  (фаза 180°)
// => на двух пинах строго противофазный меандр 50% без аппаратной инверсии выхода.
//
// При аппаратной синхронизации несущая идёт НЕПРЕРЫВНО; открывает её путь на пьезо
// сигнал EXTTXE от DWM1000 через ключи SN74HC4066 (см. dwm1000.c, config.h).
//
#include "carrier.h"
#include "config.h"
#include "dwm1000_pins.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

#define CARRIER_RES_HZ       160000000u  // разрешение таймера MCPWM = 160 МГц (PLL_F160M, тик 6.25 нс)
                                         // -> на 25 кГц период ≈ 6400 тиков, шаг по f ≈ 3.9 Гц
#define CARRIER_PERIOD_TICKS (CARRIER_RES_HZ / CARRIER_HZ)

// Делитель 0–3.3 В подстройки частоты. GPIO35 на ESP32-S3 НЕ имеет АЦП (он только на GPIO1–20),
// поэтому используется GPIO1 = ADC1_CH0.
#define FREQ_TRIM_ADC_UNIT     ADC_UNIT_1
#define FREQ_TRIM_ADC_CHANNEL  ADC_CHANNEL_0   // GPIO1
#define FREQ_TRIM_SAMPLES      64u             // усреднение для подавления шума АЦП
#define FREQ_TRIM_PERIOD_MS    100u            // период опроса делителя
#define FREQ_TRIM_DEADBAND_HZ  2u              // не дёргать таймер при дрожании < 2 Гц

static const char *TAG = "carrier";

// Дескрипторы MCPWM, нужны для смены частоты на лету.
static mcpwm_timer_handle_t s_timer = NULL;
static mcpwm_cmpr_handle_t  s_cmp   = NULL;

esp_err_t carrier_init(void)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = CARRIER_RES_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = CARRIER_PERIOD_TICKS,
        // Новый период вступает в силу на counter==0 (как и compare ниже) -> смена частоты
        // на лету без срыва фазы и без укорочённого "обрезка" текущего периода.
        .flags.update_period_on_empty = true,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &timer), TAG, "new_timer");

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &oper), TAG, "new_operator");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(oper, timer), TAG, "connect_timer");

    mcpwm_cmpr_handle_t cmp = NULL;
    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(oper, &cmp_cfg, &cmp), TAG, "new_comparator");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(cmp, CARRIER_PERIOD_TICKS / 2),
                        TAG, "set_compare");   // 50% скважность

    mcpwm_gen_handle_t gen_a = NULL, gen_b = NULL;
    mcpwm_generator_config_t gen_a_cfg = { .gen_gpio_num = CARRIER_A_PIN };
    mcpwm_generator_config_t gen_b_cfg = { .gen_gpio_num = CARRIER_B_PIN };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(oper, &gen_a_cfg, &gen_a), TAG, "gen_a");
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(oper, &gen_b_cfg, &gen_b), TAG, "gen_b");

    // genA: фаза 0° — HIGH в начале периода, LOW по компаратору
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)),
        TAG, "a_timer_act");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(gen_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_LOW)),
        TAG, "a_cmp_act");

    // genB: фаза 180° — LOW в начале периода, HIGH по компаратору (инверсия genA)
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)),
        TAG, "b_timer_act");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(gen_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_HIGH)),
        TAG, "b_cmp_act");

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(timer), TAG, "timer_enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP), TAG, "timer_start");

    s_timer = timer;   // сохранить для carrier_set_freq()
    s_cmp   = cmp;

    ESP_LOGI(TAG, "carrier running continuously: %u Hz on GPIO%d/%d (gated by EXTTXE)",
             (unsigned)CARRIER_HZ, CARRIER_A_PIN, CARRIER_B_PIN);
    return ESP_OK;
}

esp_err_t carrier_set_freq(uint32_t hz)
{
    if (s_timer == NULL || s_cmp == NULL || hz == 0)
        return ESP_ERR_INVALID_STATE;

    uint32_t period = CARRIER_RES_HZ / hz;
    if (period < 2) period = 2;   // защита от деления в ноль скважности

    // Обе записи применяются на counter==0 (timer: update_period_on_empty, cmp: update_cmp_on_tez),
    // поэтому 50%-скважность и противофазность сохраняются при смене частоты.
    ESP_RETURN_ON_ERROR(mcpwm_timer_set_period(s_timer, period), TAG, "set_period");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp, period / 2), TAG, "set_cmp");
    return ESP_OK;
}

// Фоновая задача: читает делитель 0–3.3 В на GPIO1 и подстраивает частоту несущей
// в диапазоне CARRIER_HZ ± CARRIER_TRIM_HZ. Усреднение + дедбенд гасят дрожание АЦП.
static void freq_trim_task(void *arg)
{
    adc_oneshot_unit_handle_t adc = NULL;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = FREQ_TRIM_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,        // полный диапазон входа ~0..3.3 В
        .bitwidth = ADC_BITWIDTH_DEFAULT,   // 12 бит -> raw 0..4095
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, FREQ_TRIM_ADC_CHANNEL, &chan_cfg));

    ESP_LOGI(TAG, "freq trim: %u Hz ± %u Hz по делителю на GPIO1 (ADC1_CH0)",
             (unsigned)CARRIER_HZ, (unsigned)CARRIER_TRIM_HZ);

    uint32_t applied = CARRIER_HZ;
    unsigned tick = 0;
    const unsigned log_every = 1000u / FREQ_TRIM_PERIOD_MS;   // лог частоты раз в секунду
    for (;;) {
        int acc = 0;
        for (unsigned i = 0; i < FREQ_TRIM_SAMPLES; i++) {
            int raw = 0;
            if (adc_oneshot_read(adc, FREQ_TRIM_ADC_CHANNEL, &raw) == ESP_OK)
                acc += raw;
        }
        int raw = acc / (int)FREQ_TRIM_SAMPLES;                 // 0..4095, центр ~2048
        int delta = (raw - 2048) * (int)CARRIER_TRIM_HZ / 2048; // ≈ −CARRIER_TRIM_HZ..+CARRIER_TRIM_HZ
        uint32_t target = (uint32_t)((int)CARRIER_HZ + delta);

        uint32_t diff = (target > applied) ? (target - applied) : (applied - target);
        if (diff >= FREQ_TRIM_DEADBAND_HZ) {
            if (carrier_set_freq(target) == ESP_OK)
                applied = target;
        }

        if (++tick >= log_every) {            // раз в секунду печатаем текущую частоту
            tick = 0;
            ESP_LOGI(TAG, "несущая %u Гц (raw=%d)", (unsigned)applied, raw);
        }
        vTaskDelay(pdMS_TO_TICKS(FREQ_TRIM_PERIOD_MS));
    }
}

void carrier_start_freq_trim(void)
{
    xTaskCreate(freq_trim_task, "freq_trim", 4096, NULL, 4, NULL);
}
