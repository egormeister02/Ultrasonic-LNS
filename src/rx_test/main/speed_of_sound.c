//
// Скорость звука во влажном воздухе. Порт scripts/speed_of_sound.py (Cramer 1993,
// модель идеального газа без вириальной поправки):
//   c(T,p,h) = sqrt(gamma * R_mix * T),  x_v = h * p_sat(T) / p
//
#include "speed_of_sound.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "sos";

#define R_GAS   8.314462618    // Дж/(моль·К)
#define M_D     0.0289652      // кг/моль, сухой воздух
#define M_V     0.01801528     // кг/моль, вода
#define CP_D    1005.0         // Дж/(кг·К), сухой воздух при p=const
#define CP_V    1850.0         // Дж/(кг·К), водяной пар при p=const

double p_sat_pa(double T)
{
    return 611.21 * exp(17.502 * (T - 273.15) / (T - 32.19));
}

double speed_of_sound(double T, double p, double h)
{
    double x_v = h * p_sat_pa(T) / p;
    double M_mix = (1.0 - x_v) * M_D + x_v * M_V;
    double R_mix = R_GAS / M_mix;
    double cp_mix = (1.0 - x_v) * CP_D + x_v * CP_V;
    double gamma = cp_mix / (cp_mix - R_mix);
    return sqrt(gamma * R_mix * T);
}

void speed_of_sound_selfcheck(void)
{
    struct { const char *label; double T, p, h; } cases[] = {
        { "dry, 0 C, 1 atm",      273.15, 101325, 0.0 },
        { "dry, 20 C, 1 atm",     293.15, 101325, 0.0 },
        { "50% RH, 20 C",         293.15, 101325, 0.5 },
        { "100% RH, 20 C",        293.15, 101325, 1.0 },
        { "dry, 20 C, 86 kPa",    293.15,  86500, 0.0 },
        { "50% RH, 20 C, 86 kPa", 293.15,  86500, 0.5 },
    };
    ESP_LOGI(TAG, "selfcheck (сверка с speed_of_sound.py):");
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        double c = speed_of_sound(cases[i].T, cases[i].p, cases[i].h);
        double x_v = cases[i].h * p_sat_pa(cases[i].T) / cases[i].p;
        ESP_LOGI(TAG, "  %-24s c=%8.3f m/s  x_v=%.5f", cases[i].label, c, x_v);
    }
}
