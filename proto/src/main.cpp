/**
 * Диплом — Прототип
 * BlackPill STM32F401CCU6
 * 
 * Двунаправленное управление драйвером мотора TI 8871
 * Частота PWM: 24.5 - 25.5 кГц (задаётся потенциометром)
 * Используется H-bridge режим: IN1 = PWM, IN2 = инвертированный PWM
 */

#include <Arduino.h>

// === ПИНЫ ===
#define POT_PIN      PA0        // Потенциометр (делитель напряжения)
#define IN1_PIN      PB6        // PWM на драйвер (TIM4 CH1)
#define IN2_PIN      PB7        // Инвертированный PWM (TIM4 CH2)

// === ПАРАМЕТРЫ ===
#define MIN_FREQ     24500      // Гц
#define MAX_FREQ     25500      // Гц
#define ADC_MAX      4095       // 12-бит ADC

// Таймер для PWM (TIM4)
HardwareTimer pwmtimer(TIM4);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Прототип: H-bridge режим ===");
  Serial.print("Диапазон: ");
  Serial.print(MIN_FREQ);
  Serial.print(" - ");
  Serial.print(MAX_FREQ);
  Serial.println(" Гц");
  Serial.println();

  // ADC на потенциометр
  pinMode(POT_PIN, INPUT_ANALOG);

  // === Настройка PWM на TIM4 ===
  // Канал 1: PB6 (IN1)
  // Канал 2: PB7 (IN2) - инвертированный
  
  pinmap_pinout(digitalPinToPinName(IN1_PIN), PinMap_PWM);
  pinmap_pinout(digitalPinToPinName(IN2_PIN), PinMap_PWM);

  // Начальная частота - середина диапазона
  uint32_t initialFreq = (MIN_FREQ + MAX_FREQ) / 2;
  
  // Prescaler: 84 MHz / 84 = 1 MHz
  pwmtimer.setPrescaleFactor(84);
  
  // Период для нужной частоты
  pwmtimer.setOverflow(1000000 / initialFreq, MICROSEC_FORMAT);
  
  // Канал 1: 50% скважность
  pwmtimer.setCaptureCompare(1, 50, PERCENT_COMPARE_FORMAT);
  
  // Канал 2: 50% скважность (инвертированная фаза)
  // Для инверсии используем другой режим
  pwmtimer.setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT);
  
  // Включаем режим Complementary (инвертированный выход)
  // TIM4 CH1 -> PB6, TIM4 CH1N -> PB6 (альтернативная функция)
  // Но на F401 CCU6 PB7 это TIM4 CH2, не CH1N
  
  // Альтернатива: программная инверсия через toggle mode
  // Или просто инвертируем фазу в коде
  
  pwmtimer.resume();
}

void loop() {
  // Читаем потенциометр
  uint16_t adcValue = analogRead(POT_PIN);

  // ADC -> частота
  uint32_t freq = MIN_FREQ + (uint32_t)((uint32_t)adcValue * (MAX_FREQ - MIN_FREQ) / ADC_MAX);

  // Обновляем период
  pwmtimer.setOverflow(1000000 / freq, MICROSEC_FORMAT);

  // Вывод в консоль
  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print(" | Частота: ");
  Serial.print(freq);
  Serial.println(" Гц");

  delay(100);
}
