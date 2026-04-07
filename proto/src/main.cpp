/**
 * Диплом — Прототип
 * BlackPill STM32F401CCU6
 * 
 * Управление драйвером мотора TI 8871
 * Частота PWM: 24.5 - 25.5 кГц (задаётся потенциометром)
 */

#include <Arduino.h>

// === ПИНЫ ===
#define POT_PIN      PA0        // Потенциометр (делитель напряжения)
#define PWM_PIN      PB6        // Выход PWM на драйвер (TIM4 CH1)

// === ПАРАМЕТРЫ ===
#define MIN_FREQ     24500      // Гц
#define MAX_FREQ     25500      // Гц
#define ADC_MAX      4095       // 12-бит ADC

// Таймер для PWM
HardwareTimer pwmtimer(TIM4);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Прототип: Настройка частоты драйвера ===");
  Serial.print("Диапазон: ");
  Serial.print(MIN_FREQ);
  Serial.print(" - ");
  Serial.print(MAX_FREQ);
  Serial.println(" Гц");
  Serial.println();

  // ADC на потенциометр (INPUT_ANALOG включает АЦП)
  pinMode(POT_PIN, INPUT_ANALOG);

  // === Настройка PWM на TIM4 ===
  // STM32F401: APB1 = 84MHz
  // Для получения нужной частоты настраиваем таймер вручную
  
  // Переназначаем пин на таймер
  pinmap_pinout(digitalPinToPinName(PWM_PIN), PinMap_PWM);
  
  // Начальная частота - середина диапазона
  uint32_t initialFreq = (MIN_FREQ + MAX_FREQ) / 2;
  pwmtimer.setPrescaleFactor(84);        // 84 MHz / 84 = 1 MHz
  pwmtimer.setOverflow(1000000 / initialFreq, MICROSEC_FORMAT); // период в мкс
  pwmtimer.setCaptureCompare(1, 50, PERCENT_COMPARE_FORMAT);   // 50% скважность
  pwmtimer.attachInterrupt(TIMER_CH1, []() {});
  pwmtimer.resume();  // Запуск таймера
}

void loop() {
  // Читаем потенциометр (0 - 4095)
  uint16_t adcValue = analogRead(POT_PIN);

  // Карта: ADC → частота
  // Линейная интерполяция
  uint32_t freq = MIN_FREQ + (uint32_t)((uint32_t)adcValue * (MAX_FREQ - MIN_FREQ) / ADC_MAX);

  // Обновляем PWM
  pwmtimer.setOverflow(1000000 / freq, MICROSEC_FORMAT);

  // Вывод в консоль
  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print(" | Частота: ");
  Serial.print(freq);
  Serial.println(" Гц");

  delay(100);  // Небольшая задержка, чтобы не спамить
}
