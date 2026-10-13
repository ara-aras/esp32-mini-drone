#include "battery.h"
#include "config.h"
#include <Arduino.h>

namespace {
volatile float vbat = 0;
#if VBAT_SENSE_ENABLED
bool primed = false;
#endif
}

void batteryBegin() {
#if VBAT_SENSE_ENABLED
  analogReadResolution(12);
  pinMode(PIN_VBAT, INPUT);
#endif
}

void batteryUpdate() {
#if VBAT_SENSE_ENABLED
  float v = analogReadMilliVolts(PIN_VBAT) / 1000.0f * VBAT_DIVIDER * VBAT_CALIBRATION;
  if (!primed) { vbat = v; primed = true; }
  else vbat = vbat + 0.2f * (v - vbat);
#endif
}

float batteryVoltage() { return vbat; }

bool batteryOkToArm() {
#if VBAT_SENSE_ENABLED
  return vbat >= VBAT_MIN_ARM;
#else
  return true;
#endif
}

bool batteryLow() {
#if VBAT_SENSE_ENABLED
  return vbat < VBAT_LOW_WARN;
#else
  return false;
#endif
}
