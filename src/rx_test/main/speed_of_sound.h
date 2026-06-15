#pragma once
//
// Скорость звука во влажном воздухе — модель идеального газа (Cramer 1993 без
// вириальной поправки), портирована из scripts/speed_of_sound.py. Соответствует
// формуле eq:speed-of-sound-humid-air в ultrasonic-navigation.tex.
//

// Давление насыщенного водяного пара (Tetens), T в кельвинах, возвращает Па.
double p_sat_pa(double T_kelvin);

// Скорость звука [м/с]. T — кельвины, p — Па, h — отн. влажность 0..1.
double speed_of_sound(double T_kelvin, double p_pa, double h_rel);

// Самопроверка: печатает таблицу эталонных случаев (сверка с Python-версией).
void speed_of_sound_selfcheck(void);
