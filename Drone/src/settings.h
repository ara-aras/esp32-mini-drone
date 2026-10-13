#pragma once
#include <Arduino.h>

// Runtime-tunable values. Defaults come from config.h; `save` stores them in flash.
struct Settings {
  float rpKp, rpKi, rpKd;
  float yawKp, yawKi, yawKd;
  float angleKp, maxAngle, maxRateRp, maxRateYaw;
  float motorIdle, testLevel;
  float trimRoll, trimPitch;
};

void     settingsBegin();          // load from flash, or defaults
Settings settingsGet();            // thread-safe snapshot
uint32_t settingsVersion();        // changes on every modification
bool     settingsSet(const char *key, float value, String &err);
bool     settingsSave();
bool     settingsLoad();
void     settingsDefaults();
String   settingsList();           // human-readable
String   settingsJson();
