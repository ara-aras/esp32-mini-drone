#pragma once
#include "config.h"
#include <cstdint>
#include <cstdlib>
#include <cmath>

// -500..500 stick value -> -1..1 with deadband (host-testable).
inline float rcStick(int16_t v) {
  float a = (float)abs((int)v);
  if (a < STICK_DEADBAND) return 0.0f;
  float r = (a - STICK_DEADBAND) / (500.0f - STICK_DEADBAND);
  if (r > 1.0f) r = 1.0f;
  return v < 0 ? -r : r;
}

// 0..1000 -> 0..1
inline float rcThrottle(int16_t v) {
  if (v < 0) v = 0;
  if (v > 1000) v = 1000;
  return v / 1000.0f;
}

inline float rcClampUnit(float v) {
  if (!std::isfinite(v)) return 0.0f;
  return v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
}

// ESP-Drone app commander -> normalised sticks (roll+ right, pitch+ forward, yaw+ right).
struct RcSticks {
  float throttle, roll, pitch, yaw;
};

inline RcSticks rcFromApp(float rollDeg, float pitchDeg, float yawDps, uint16_t thrust) {
  RcSticks r;
  r.throttle = thrust < APP_MIN_THRUST ? 0.0f
             : (thrust >= APP_THRUST_FULL ? 1.0f : (float)thrust / APP_THRUST_FULL);
  r.roll  = rcClampUnit(rollDeg / APP_FULL_ANGLE_DEG);
  r.pitch = rcClampUnit(-pitchDeg / APP_FULL_ANGLE_DEG);  // nose-up target = stick back
  r.yaw   = rcClampUnit(yawDps / APP_FULL_YAW_DPS);
  return r;
}
