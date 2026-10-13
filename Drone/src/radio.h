#pragma once
#include <Arduino.h>

// ESP-NOW wire format — keep IDENTICAL to Transmitter.ino.
struct __attribute__((packed)) RcPacket {
  uint16_t magic;     // RADIO_MAGIC
  uint16_t seq;
  int16_t  throttle;  // 0..1000
  int16_t  roll;      // -500..500, + = stick right
  int16_t  pitch;     // -500..500, + = stick forward
  int16_t  yaw;       // -500..500, + = stick right
  uint8_t  flags;     // bit0 = arm switch
};
#define RC_FLAG_ARM 0x01

enum class RcSource : uint8_t { None = 0, EspNow = 1, App = 2 };

struct RcInput {
  float    throttle;          // 0..1
  float    roll, pitch, yaw;  // -1..1
  bool     armSwitch;         // ESP-NOW: switch. App: thrust raised (after a zero)
  bool     armImpliesLowThrottle;  // app: arming edge itself proves throttle was zero
  bool     valid;             // at least one packet from the selected source
  uint32_t ageMs;             // time since its last packet
  uint32_t packets;
  RcSource source;
};

bool radioBegin();            // ESP-NOW (if enabled). Call after netBegin().
void radioGet(RcInput &out);
void radioFeedApp(float rollDeg, float pitchDeg, float yawDps, uint16_t thrust);  // from app link
const char *rcSourceName(RcSource s);
