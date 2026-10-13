#pragma once
// ESP-Drone app "arm switch" emulation (host-testable, header only).
// The app has no arm button. Like ESP-Drone's thrustLocked: after (re)connecting,
// thrust must be seen at zero once; raising thrust then counts as "switch on";
// thrust back at zero for APP_DISARM_HOLD_MS counts as "switch off".
#include <cstdint>
#include "config.h"

struct AppArm {
  uint32_t lastMs = 0;        // last commander packet
  uint32_t lastThrustMs = 0;  // last packet with thrust > 0 while unlocked
  bool any = false;
  bool unlocked = false;      // a zero-thrust packet was seen since the link came up

  void feed(uint16_t thrust, uint32_t now) {
    if (!any || now - lastMs > FAILSAFE_TIMEOUT_MS) unlocked = false;  // (re)connect
    if (thrust < APP_MIN_THRUST) unlocked = true;
    else if (unlocked) lastThrustMs = now;
    lastMs = now;
    any = true;
  }

  // "Switch on" = unlocked and thrust seen above zero within the hold window.
  bool armSwitch(uint32_t now) const {
    return unlocked && lastThrustMs != 0 && (now - lastThrustMs) < APP_DISARM_HOLD_MS;
  }
};
