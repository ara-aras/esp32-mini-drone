#include "motors.h"
#include "config.h"
#include <esp_arduino_version.h>

namespace {
// PWM pin per motor. With DRV8833 a reversed motor gets the PWM on its IN2
// pin and IN1 held low, so PINS[] is resolved at init from MOTOR_REVERSE.
int PINS[4] = {PIN_MOTOR1, PIN_MOTOR2, PIN_MOTOR3, PIN_MOTOR4};

#if MOTOR_TYPE == MOTOR_TYPE_BRUSHED
const uint32_t FREQ = BRUSHED_PWM_FREQ;
const uint8_t  BITS = BRUSHED_PWM_BITS;
#else
const uint32_t FREQ = ESC_PWM_FREQ;
const uint8_t  BITS = ESC_PWM_BITS;
#endif
const uint32_t MAX_DUTY = (1UL << BITS) - 1;

uint32_t dutyFor(float lvl) {
  if (lvl < 0) lvl = 0;
  if (lvl > 1) lvl = 1;
#if MOTOR_TYPE == MOTOR_TYPE_BRUSHED
  return (uint32_t)(lvl * MAX_DUTY);
#else
  float us = ESC_MIN_US + lvl * (ESC_MAX_US - ESC_MIN_US);
  float periodUs = 1000000.0f / FREQ;
  return (uint32_t)(us / periodUs * (MAX_DUTY + 1));
#endif
}

void writeDuty(int idx, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PINS[idx], duty);
#else
  ledcWrite(idx, duty);  // channel == motor index on core 2.x
#endif
}
}  // namespace

bool motorsBegin() {
  bool ok = true;
#if USE_DRV8833 && MOTOR_TYPE == MOTOR_TYPE_BRUSHED
  if (PIN_DRV_SLEEP >= 0) {           // keep driver asleep while we set up
    pinMode(PIN_DRV_SLEEP, OUTPUT);
    digitalWrite(PIN_DRV_SLEEP, LOW);
  }
  int in2[4] = {PIN_MOTOR1_IN2, PIN_MOTOR2_IN2, PIN_MOTOR3_IN2, PIN_MOTOR4_IN2};
  const int rev[4] = MOTOR_REVERSE;
  for (int i = 0; i < 4; i++) {
    if (rev[i] && in2[i] >= 0) {   // swap roles: PWM on IN2, IN1 held low
      int t = PINS[i]; PINS[i] = in2[i]; in2[i] = t;
    }
    if (in2[i] >= 0) {
      pinMode(in2[i], OUTPUT);
      digitalWrite(in2[i], LOW);
    }
  }
#endif
  for (int i = 0; i < 4; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ok &= ledcAttach(PINS[i], FREQ, BITS);
#else
    ledcSetup(i, FREQ, BITS);
    ledcAttachPin(PINS[i], i);
#endif
  }
  motorsStop();
#if USE_DRV8833 && MOTOR_TYPE == MOTOR_TYPE_BRUSHED
  if (PIN_DRV_SLEEP >= 0) {
    delay(2);
    digitalWrite(PIN_DRV_SLEEP, HIGH);  // wake driver; outputs are at 0 %
  }
#endif
  return ok;
}

void motorsWriteOne(int idx, float lvl) {
  if (idx < 0 || idx > 3) return;
  writeDuty(idx, dutyFor(lvl));
}

void motorsWrite(const float lvl[4]) {
  for (int i = 0; i < 4; i++) writeDuty(i, dutyFor(lvl[i]));
}

void motorsStop() {
  for (int i = 0; i < 4; i++) writeDuty(i, dutyFor(0));
}
