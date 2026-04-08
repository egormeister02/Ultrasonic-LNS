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
#define ADC_MAX      1023       // 10-бит ADC

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

  // === Настройка пинов как альтернативная функция для PWM ===
  pinMode(IN1_PIN, OUTPUT_AF);
  pinMode(IN2_PIN, OUTPUT_AF);

  // === Настройка таймера TIM4 ===
  // Частота тактирования APB1 = 84 MHz
  // Prescaler = 84 -> 1 MHz тиков
  pwmtimer.setPrescaleFactor(84);
  
  // Начальная частота - середина диапазона
  uint32_t initialFreq = (MIN_FREQ + MAX_FREQ) / 2;
  
  // Период для нужной частоты (в микросекундах)
  pwmtimer.setOverflow(1000000 / initialFreq, MICROSEC_FORMAT);
  
  // Режим PWM для каналов
  pwmtimer.setMode(1, PWM, IN1_PIN);   // CH1 -> PB6
  pwmtimer.setMode(2, PWM, IN2_PIN);   // CH2 -> PB7
  
  // Скважность 50%
  pwmtimer.setCaptureCompare(1, 50, PERCENT_COMPARE_FORMAT);  // PB6: 50%
  pwmtimer.setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT);  // PB7: 50%
  
  // Запуск таймера
  pwmtimer.resume();
  
  Serial.println("PWM запущен");
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
