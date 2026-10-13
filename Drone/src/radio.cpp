#include "radio.h"
#include "config.h"
#include "rcmath.h"
#include "apparm.h"
#include "logbuf.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

namespace {
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ESP-NOW transmitter
RcPacket last = {};
uint32_t lastMs = 0;
uint32_t count = 0;
bool gotAny = false;

// ESP-Drone app
float appRoll = 0, appPitch = 0, appYaw = 0;
uint16_t appThrust = 0;
uint32_t appCount = 0;
AppArm appArm;  // thrustLocked emulation, see apparm.h

void handle(const uint8_t *data, int len) {
  if (len != sizeof(RcPacket)) return;
  RcPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != RADIO_MAGIC) return;
  portENTER_CRITICAL(&mux);
  last = p;
  lastMs = millis();
  count++;
  gotAny = true;
  portEXIT_CRITICAL(&mux);
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onRecv(const esp_now_recv_info_t *, const uint8_t *data, int len) { handle(data, len); }
#else
void onRecv(const uint8_t *, const uint8_t *data, int len) { handle(data, len); }
#endif
}  // namespace

const char *rcSourceName(RcSource s) {
  switch (s) {
    case RcSource::EspNow: return "ESP-NOW";
    case RcSource::App: return "App";
    default: return "none";
  }
}

bool radioBegin() {
#if RADIO_ESPNOW_ENABLED
  if (WiFi.getMode() == WIFI_MODE_NULL) WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(RADIO_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    dlog("[radio] esp_now_init failed\n");
    return false;
  }
  esp_now_register_recv_cb(onRecv);
  dlog("[radio] ESP-NOW ready, channel %d, STA MAC %s\n", RADIO_WIFI_CHANNEL,
       WiFi.macAddress().c_str());
#endif
  return true;
}

void radioFeedApp(float r, float p, float y, uint16_t t) {
  uint32_t now = millis();
  portENTER_CRITICAL(&mux);
  appArm.feed(t, now);
  appRoll = r; appPitch = p; appYaw = y; appThrust = t;
  appCount++;
  portEXIT_CRITICAL(&mux);
}

void radioGet(RcInput &out) {
  portENTER_CRITICAL(&mux);
  RcPacket p = last;
  uint32_t t = lastMs, n = count;
  bool any = gotAny;
  float ar = appRoll, ap = appPitch, ay = appYaw;
  uint16_t at = appThrust;
  uint32_t aN = appCount;
  AppArm aa = appArm;
  portEXIT_CRITICAL(&mux);

  uint32_t now = millis();
  out = RcInput{};
  bool espFresh = any && (now - t) < FAILSAFE_TIMEOUT_MS;

  if (any && (espFresh || !aa.any)) {
    out.source = RcSource::EspNow;
    out.valid = true;
    out.packets = n;
    out.ageMs = now - t;
    out.throttle = rcThrottle(p.throttle);
    out.roll = rcStick(p.roll);
    out.pitch = rcStick(p.pitch);
    out.yaw = rcStick(p.yaw);
    out.armSwitch = (p.flags & RC_FLAG_ARM) != 0;
    out.armImpliesLowThrottle = false;
  } else if (aa.any) {
    out.source = RcSource::App;
    out.valid = true;
    out.packets = aN;
    out.ageMs = now - aa.lastMs;
    RcSticks s = rcFromApp(ar, ap, ay, at);
    out.throttle = s.throttle;
    out.roll = s.roll;
    out.pitch = s.pitch;
    out.yaw = s.yaw;
    out.armSwitch = aa.armSwitch(now);
    out.armImpliesLowThrottle = true;
  } else {
    out.source = RcSource::None;
    out.valid = false;
    out.ageMs = UINT32_MAX;
  }
}
