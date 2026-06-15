//
// Тест приёмного (земляного) узла системы УЗ-навигации.
//   1) BME280 (I2C): температура, давление, влажность;
//   2) расчёт скорости звука по модели влажного воздуха;
//   3) приём UWB-сообщений по DWM1000  — ДОБАВЛЯЕТСЯ отдельно (см. dwm1000_rx).
//
#include "config.h"
#include "bme280.h"
#include "speed_of_sound.h"
#include "dwm1000_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "rx_test";

void app_main(void)
{
    ESP_LOGI(TAG, "RX test: BME280 + speed of sound + DWM1000 RX");

    // 1) сверка формулы скорости звука с эталоном (Python)
    speed_of_sound_selfcheck();

    // 2) датчик
    bool sensor_ok = (bme280_init() == ESP_OK);
    if (!sensor_ok) {
        ESP_LOGE(TAG, "BME280 не найден — проверь I2C (SDA=%d SCL=%d, адрес 0x%02X)",
                 I2C_SDA_PIN, I2C_SCL_PIN, BME280_ADDR);
    }

    // 3) приём UWB по DWM1000 (отдельная задача)
    if (dwm1000_rx_init() == ESP_OK) {
        dwm1000_rx_start();
    } else {
        ESP_LOGE(TAG, "DWM1000 RX init failed — приём UWB недоступен");
    }

    while (true) {
        if (sensor_ok) {
            bme280_data_t m;
            if (bme280_read(&m) == ESP_OK) {
                double T_k = m.temperature_c + 273.15;
                double h   = m.humidity_rh / 100.0;
                double c   = speed_of_sound(T_k, m.pressure_pa, h);
                ESP_LOGI(TAG, "T=%.2f C  p=%.1f Pa  RH=%.1f %%  ->  c = %.3f m/s",
                         m.temperature_c, m.pressure_pa, m.humidity_rh, c);
            } else {
                ESP_LOGW(TAG, "BME280 read error");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MEAS_PERIOD_MS));
    }
}
