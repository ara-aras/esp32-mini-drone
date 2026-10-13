#include "settings.h"
#include "config.h"
#include "logbuf.h"
#include <Preferences.h>

namespace {
struct KeyDef {
  const char *key;
  float Settings::*ptr;
  float lo, hi;
};

const KeyDef KEYS[] = {
    {"rp_kp", &Settings::rpKp, 0, 0.02f},
    {"rp_ki", &Settings::rpKi, 0, 0.05f},
    {"rp_kd", &Settings::rpKd, 0, 0.001f},
    {"yaw_kp", &Settings::yawKp, 0, 0.03f},
    {"yaw_ki", &Settings::yawKi, 0, 0.05f},
    {"yaw_kd", &Settings::yawKd, 0, 0.001f},
    {"angle_kp", &Settings::angleKp, 0, 20},
    {"max_angle", &Settings::maxAngle, 5, 60},
    {"max_rate_rp", &Settings::maxRateRp, 30, 720},
    {"max_rate_yaw", &Settings::maxRateYaw, 30, 720},
    {"motor_idle", &Settings::motorIdle, 0, 0.2f},
    {"test_level", &Settings::testLevel, 0.05f, 0.35f},
    {"trim_roll", &Settings::trimRoll, -15, 15},
    {"trim_pitch", &Settings::trimPitch, -15, 15},
};
const size_t NKEYS = sizeof(KEYS) / sizeof(KEYS[0]);
// Bump SETTINGS_VERSION whenever Settings fields are reordered or change meaning.
// sizeof() alone only catches additions/removals.
const uint32_t SETTINGS_VERSION = 1;
const uint32_t SCHEMA = 0x53450000u | (SETTINGS_VERSION << 8) | sizeof(Settings);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Settings cur;
uint32_t ver = 1;

Settings makeDefaults() {
  Settings s;
  s.rpKp = RATE_RP_KP;   s.rpKi = RATE_RP_KI;   s.rpKd = RATE_RP_KD;
  s.yawKp = RATE_YAW_KP; s.yawKi = RATE_YAW_KI; s.yawKd = RATE_YAW_KD;
  s.angleKp = ANGLE_KP;  s.maxAngle = MAX_ANGLE_DEG;
  s.maxRateRp = MAX_RATE_RP_DPS; s.maxRateYaw = MAX_RATE_YAW_DPS;
  s.motorIdle = MOTOR_IDLE; s.testLevel = MOTOR_TEST_LEVEL;
  s.trimRoll = ACC_TRIM_ROLL_DEG; s.trimPitch = ACC_TRIM_PITCH_DEG;
  return s;
}

bool valid(const Settings &s) {
  for (size_t i = 0; i < NKEYS; i++) {
    float v = s.*(KEYS[i].ptr);
    if (!(v >= KEYS[i].lo && v <= KEYS[i].hi)) return false;  // also rejects NaN
  }
  return true;
}

void replace(const Settings &s) {
  portENTER_CRITICAL(&mux);
  cur = s;
  ver++;
  portEXIT_CRITICAL(&mux);
}
}  // namespace

void settingsBegin() {
  cur = makeDefaults();
  if (settingsLoad()) dlog("[settings] loaded from flash\n");
  else dlog("[settings] using defaults from config.h\n");
}

Settings settingsGet() {
  portENTER_CRITICAL(&mux);
  Settings s = cur;
  portEXIT_CRITICAL(&mux);
  return s;
}

uint32_t settingsVersion() {
  portENTER_CRITICAL(&mux);
  uint32_t v = ver;
  portEXIT_CRITICAL(&mux);
  return v;
}

bool settingsSet(const char *key, float value, String &err) {
  for (size_t i = 0; i < NKEYS; i++) {
    if (strcmp(key, KEYS[i].key) != 0) continue;
    if (!(value >= KEYS[i].lo && value <= KEYS[i].hi)) {
      err = String(key) + " must be between " + String(KEYS[i].lo, 5) + " and " +
            String(KEYS[i].hi, 5);
      return false;
    }
    Settings s = settingsGet();
    s.*(KEYS[i].ptr) = value;
    replace(s);
    return true;
  }
  err = String("unknown key '") + key + "' (try: get)";
  return false;
}

bool settingsSave() {
  Settings s = settingsGet();
  Preferences p;
  if (!p.begin("drone", false)) return false;
  bool ok = p.putUInt("schema", SCHEMA) > 0;
  ok &= p.putBytes("s", &s, sizeof(s)) == sizeof(s);
  p.end();
  return ok;
}

bool settingsLoad() {
  Preferences p;
  if (!p.begin("drone", true)) return false;
  Settings s;
  bool ok = p.getUInt("schema", 0) == SCHEMA &&
            p.getBytesLength("s") == sizeof(s) &&
            p.getBytes("s", &s, sizeof(s)) == sizeof(s);
  p.end();
  if (!ok || !valid(s)) return false;
  replace(s);
  return true;
}

void settingsDefaults() { replace(makeDefaults()); }

String settingsList() {
  Settings s = settingsGet();
  String out;
  char line[64];
  for (size_t i = 0; i < NKEYS; i++) {
    snprintf(line, sizeof(line), "  %-13s %.6g\n", KEYS[i].key, s.*(KEYS[i].ptr));
    out += line;
  }
  return out;
}

String settingsJson() {
  Settings s = settingsGet();
  String out = "{";
  char item[64];
  for (size_t i = 0; i < NKEYS; i++) {
    snprintf(item, sizeof(item), "%s\"%s\":%.6g", i ? "," : "", KEYS[i].key, s.*(KEYS[i].ptr));
    out += item;
  }
  return out + "}";
}
