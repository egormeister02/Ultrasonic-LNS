/**
 * Diploma — Prototype
 * BlackPill STM32F401CCU6
 *
 * Bidirectional motor driver control (TI DRV8871)
 * PWM frequency: 24.5 - 25.5 kHz (set by potentiometer)
 * H-bridge mode: IN1 = PWM, IN2 = inverted PWM
 */

#include <Arduino.h>

#define POT_PIN      PA0
#define IN1_PIN      PB6        // TIM4 CH1
#define IN2_PIN      PB7        // TIM4 CH2

#define MIN_FREQ     24500
#define MAX_FREQ     28000
#define ADC_MAX      1023

HardwareTimer pwmTimer(TIM4);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Prototype: H-bridge mode ===");
  Serial.print("Range: ");
  Serial.print(MIN_FREQ);
  Serial.print(" - ");
  Serial.print(MAX_FREQ);
  Serial.println(" Hz");

  pinMode(POT_PIN, INPUT_ANALOG);

  uint32_t initialFreq = (MIN_FREQ + MAX_FREQ) / 2;

  pwmTimer.setOverflow(initialFreq, HERTZ_FORMAT);

  // CH1: normal PWM, CH2: inverted PWM (180° phase shift)
  pwmTimer.setMode(1, TIMER_OUTPUT_COMPARE_PWM1, IN1_PIN);
  pwmTimer.setMode(2, TIMER_OUTPUT_COMPARE_PWM2, IN2_PIN);

  pwmTimer.setCaptureCompare(1, 50, PERCENT_COMPARE_FORMAT);
  pwmTimer.setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT);

  pwmTimer.resume();

  Serial.println("PWM started");
}

void loop() {
  uint16_t adcValue = analogRead(POT_PIN);

  uint32_t freq = MIN_FREQ + (uint32_t)adcValue * (MAX_FREQ - MIN_FREQ) / ADC_MAX;

  pwmTimer.setOverflow(freq, HERTZ_FORMAT);
  // Refresh duty cycle after overflow change
  pwmTimer.setCaptureCompare(1, 50, PERCENT_COMPARE_FORMAT);
  pwmTimer.setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT);

  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print(" | Freq: ");
  Serial.print(freq);
  Serial.println(" Hz");

  delay(100);
}
