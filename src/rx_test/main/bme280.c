//
// Драйвер BME280 поверх нового i2c_master (ESP-IDF 5.x).
// Компенсация — формулы с плавающей точкой из даташита Bosch (Appendix A).
//
#include "bme280.h"
#include "config.h"
#include <string.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "bme280";

// --- регистры ---
#define REG_ID         0xD0
#define REG_RESET      0xE0
#define REG_CTRL_HUM   0xF2
#define REG_CTRL_MEAS  0xF4
#define REG_CONFIG     0xF5
#define REG_DATA       0xF7   // press(3) temp(3) hum(2)
#define REG_CALIB00    0x88   // 26 байт (T1..H1)
#define REG_CALIB26    0xE1   // 7 байт (H2..H6)

static i2c_master_dev_handle_t s_dev;

static struct {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t  H1; int16_t H2; uint8_t H3; int16_t H4, H5; int8_t H6;
    double t_fine;
} cal;

static esp_err_t rd(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 1000);
}

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, 1000);
}

static esp_err_t read_calibration(void)
{
    uint8_t a[26], b[7];
    esp_err_t e = rd(REG_CALIB00, a, sizeof(a));
    if (e != ESP_OK) return e;
    e = rd(REG_CALIB26, b, sizeof(b));
    if (e != ESP_OK) return e;

    cal.T1 = (uint16_t)(a[0] | (a[1] << 8));
    cal.T2 = (int16_t)(a[2] | (a[3] << 8));
    cal.T3 = (int16_t)(a[4] | (a[5] << 8));
    cal.P1 = (uint16_t)(a[6] | (a[7] << 8));
    cal.P2 = (int16_t)(a[8]  | (a[9] << 8));
    cal.P3 = (int16_t)(a[10] | (a[11] << 8));
    cal.P4 = (int16_t)(a[12] | (a[13] << 8));
    cal.P5 = (int16_t)(a[14] | (a[15] << 8));
    cal.P6 = (int16_t)(a[16] | (a[17] << 8));
    cal.P7 = (int16_t)(a[18] | (a[19] << 8));
    cal.P8 = (int16_t)(a[20] | (a[21] << 8));
    cal.P9 = (int16_t)(a[22] | (a[23] << 8));
    cal.H1 = a[25];
    cal.H2 = (int16_t)(b[0] | (b[1] << 8));
    cal.H3 = b[2];
    cal.H4 = (int16_t)((b[3] << 4) | (b[4] & 0x0F));
    cal.H5 = (int16_t)((b[5] << 4) | (b[4] >> 4));
    cal.H6 = (int8_t)b[6];
    return ESP_OK;
}

esp_err_t bme280_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    esp_err_t e = i2c_new_master_bus(&bus_cfg, &bus);
    if (e != ESP_OK) { ESP_LOGE(TAG, "i2c bus: %s", esp_err_to_name(e)); return e; }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    e = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (e != ESP_OK) { ESP_LOGE(TAG, "i2c dev: %s", esp_err_to_name(e)); return e; }

    uint8_t id = 0;
    e = rd(REG_ID, &id, 1);
    if (e != ESP_OK) { ESP_LOGE(TAG, "read ID: %s", esp_err_to_name(e)); return e; }
    if (id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "CHIP_ID 0x%02X (ожидался 0x%02X) — датчик не BME280 или адрес неверный",
                 id, BME280_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "CHIP_ID = 0x%02X OK", id);

    wr(REG_RESET, 0xB6);                 // soft reset
    vTaskDelay(pdMS_TO_TICKS(10));

    e = read_calibration();
    if (e != ESP_OK) { ESP_LOGE(TAG, "calib: %s", esp_err_to_name(e)); return e; }

    wr(REG_CTRL_HUM, 0x01);              // osrs_h ×1 (писать ДО ctrl_meas)
    wr(REG_CONFIG,   0xA0);              // t_standby 1000 мс, фильтр off
    wr(REG_CTRL_MEAS, 0x27);             // osrs_t ×1, osrs_p ×1, mode = normal
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

// --- компенсация (double, Bosch Appendix A) ---

static double compensate_T(int32_t adc_T)
{
    double v1 = (adc_T / 16384.0 - cal.T1 / 1024.0) * cal.T2;
    double v2 = (adc_T / 131072.0 - cal.T1 / 8192.0);
    v2 = v2 * v2 * cal.T3;
    cal.t_fine = v1 + v2;
    return cal.t_fine / 5120.0;
}

static double compensate_P(int32_t adc_P)
{
    double v1 = cal.t_fine / 2.0 - 64000.0;
    double v2 = v1 * v1 * cal.P6 / 32768.0;
    v2 = v2 + v1 * cal.P5 * 2.0;
    v2 = v2 / 4.0 + cal.P4 * 65536.0;
    v1 = (cal.P3 * v1 * v1 / 524288.0 + cal.P2 * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * cal.P1;
    if (v1 == 0.0) return 0.0;           // деление на ноль
    double p = 1048576.0 - adc_P;
    p = (p - v2 / 4096.0) * 6250.0 / v1;
    v1 = cal.P9 * p * p / 2147483648.0;
    v2 = p * cal.P8 / 32768.0;
    return p + (v1 + v2 + cal.P7) / 16.0;
}

static double compensate_H(int32_t adc_H)
{
    double h = cal.t_fine - 76800.0;
    h = (adc_H - (cal.H4 * 64.0 + cal.H5 / 16384.0 * h)) *
        (cal.H2 / 65536.0 * (1.0 + cal.H6 / 67108864.0 * h * (1.0 + cal.H3 / 67108864.0 * h)));
    h = h * (1.0 - cal.H1 * h / 524288.0);
    if (h > 100.0) h = 100.0; else if (h < 0.0) h = 0.0;
    return h;
}

esp_err_t bme280_read(bme280_data_t *out)
{
    uint8_t d[8];
    esp_err_t e = rd(REG_DATA, d, sizeof(d));
    if (e != ESP_OK) return e;

    int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    int32_t adc_H = ((int32_t)d[6] << 8) | d[7];

    out->temperature_c = (float)compensate_T(adc_T);   // задаёт t_fine, считать первым
    out->pressure_pa   = (float)compensate_P(adc_P);
    out->humidity_rh   = (float)compensate_H(adc_H);
    return ESP_OK;
}
