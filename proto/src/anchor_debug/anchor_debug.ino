/**
 * Diploma — Anchor Debug
 * BlackPill STM32F401CCU6
 *
 * Transmits a hardcoded OOK-encoded signal once per second.
 * Use this sketch to verify the anchor signal-processing chain:
 * preamp → bandpass amp → envelope detector → comparator.
 *
 * Signal: ANCHOR_ID broken into its binary representation, MSB first.
 * '1' bit = burst of FREQ_HZ carrier for BIT_DURATION_MS ms.
 * '0' bit = DRV8871 brake for BIT_DURATION_MS ms: IN1=IN2=HIGH shorts both
 *           H-bridge outputs to GND through low-side FETs, dissipating residual
 *           piezo oscillations and producing a clean zero on the receiver.
 * Between two consecutive '1' bits nothing is inserted — carrier boundary only.
 * Number of bits is derived automatically from the value:
 *   1 (0b1)   → 1 bit  → burst
 *   5 (0b101) → 3 bits → burst, brake, burst
 *   6 (0b110) → 3 bits → burst, burst, brake
 */

#include <Arduino.h>

#define IN1_PIN         PB6        // TIM4 CH1
#define IN2_PIN         PB7        // TIM4 CH2

#define FREQ_HZ         25500UL    // carrier frequency
#define BIT_DURATION_MS 2UL        // ms per bit
#define ANCHOR_ID       2          // value to transmit (use multi-bit value to test 0-bits)
#define PERIOD_MS       1000UL     // repeat period

HardwareTimer pwmTimer(TIM4);

static void setBurst(bool active) {
  if (active) {
    pwmTimer.setCaptureCompare(1, 50,  PERCENT_COMPARE_FORMAT);
    pwmTimer.setCaptureCompare(2, 50,  PERCENT_COMPARE_FORMAT);
  } else {
    // CH1(PWM1)=0% → IN1 LOW, CH2(PWM2)=100% → IN2 LOW (coast / high-Z)
    pwmTimer.setCaptureCompare(1,   0, PERCENT_COMPARE_FORMAT);
    pwmTimer.setCaptureCompare(2, 100, PERCENT_COMPARE_FORMAT);
  }
}

// IN1=HIGH, IN2=HIGH → DRV8871 brake: both H-bridge outputs pulled to GND.
// Shorts the transducer through low-side FETs (~RDS_on) so residual mechanical
// oscillations drive current into a near-short, dissipating energy quickly.
static void setBrake() {
  pwmTimer.setCaptureCompare(1, 100, PERCENT_COMPARE_FORMAT);  // PWM1@100% → always HIGH
  pwmTimer.setCaptureCompare(2,   0, PERCENT_COMPARE_FORMAT);  // PWM2@0%   → always HIGH
}

static uint8_t bitsNeeded(uint32_t value) {
  if (value == 0) return 1;
  uint8_t n = 0;
  while (value > 0) { n++; value >>= 1; }
  return n;
}

void transmitCode(uint32_t freq, uint32_t bitDurationMs, uint32_t value) {
  uint8_t numBits = bitsNeeded(value);
  pwmTimer.setOverflow(freq, HERTZ_FORMAT);
  setBurst(false);

  for (int8_t i = numBits - 1; i >= 0; i--) {
    bool bit = (value >> i) & 1;
    if (bit) setBurst(true);
    else     setBrake();
    delay(bitDurationMs);
  }

  setBurst(false);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Anchor Debug ===");
  Serial.print("ID=");  Serial.print(ANCHOR_ID, BIN);
  Serial.print(" (");   Serial.print(bitsNeeded(ANCHOR_ID));
  Serial.print(" bits, "); Serial.print(BIT_DURATION_MS);
  Serial.print(" ms/bit, "); Serial.print(FREQ_HZ);
  Serial.println(" Hz)");

  pwmTimer.setOverflow(FREQ_HZ, HERTZ_FORMAT);
  pwmTimer.setMode(1, TIMER_OUTPUT_COMPARE_PWM1, IN1_PIN);
  pwmTimer.setMode(2, TIMER_OUTPUT_COMPARE_PWM2, IN2_PIN);
  setBurst(false);
  pwmTimer.resume();
}

void loop() {
  transmitCode(FREQ_HZ, BIT_DURATION_MS, ANCHOR_ID);
  Serial.println("TX");
  delay(PERIOD_MS);
}
