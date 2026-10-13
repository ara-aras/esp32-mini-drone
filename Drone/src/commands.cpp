#include "commands.h"
#include "config.h"
#include "logbuf.h"
#include "bench.h"
#include "settings.h"
#include "telemetry.h"
#include "battery.h"
#include "net.h"
#include "radio.h"
#include "applink.h"
#include "blackbox.h"
#include <esp_arduino_version.h>

namespace {
TelemMode mode = TelemMode::Off;

void help() {
  dlog(
      "---- commands ----\n"
      "status              system summary\n"
      "tel                 toggle text telemetry (10 Hz)\n"
      "plot roll|pitch|yaw|off  Serial Plotter output (setpoint vs gyro)\n"
      "plot att            50 Hz attitude stream for tools/gyro3d.py\n"
      "selftest            sensor + logic checks\n"
      "selftest motors     ... plus motor spin check (needs props off)\n"
      "orient              guided tilt test for IMU axis signs\n"
      "cal                 recalibrate gyro (keep still)\n"
      "i2c                 scan I2C bus\n"
      "props off           CONFIRM props removed: unlock motor tests 5 min\n"
      "lock                lock motor tests again\n"
      "t1..t4              spin one motor\n"
      "mall                spin M1..M4 in order\n"
      "ramp [1-4|all]      ramp motor(s) up/down\n"
      "mix roll|pitch|yaw  show which motors the PID/mixer would speed up\n"
      "stop                abort any bench test\n"
      "bb [saved|clear]    dump flight recorder CSV (last flight / saved fault log)\n"
      "get                 list tunable settings\n"
      "set <key> <value>   change a setting (live, disarmed only)\n"
      "save | load | defaults   flash storage of settings\n"
      "reboot\n");
}

void status() {
  Telemetry t = telemetryGet();
  dlog("---- status ----\n");
  dlog("core %d.%d.%d  heap %lu  uptime %lus\n", ESP_ARDUINO_VERSION_MAJOR,
       ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH,
       (unsigned long)ESP.getFreeHeap(), (unsigned long)(millis() / 1000));
  dlog("state %s  bench %s  motor-test unlock %lus\n", t.armed ? "ARMED" : "disarmed",
       t.benchBusy ? "busy" : "idle", (unsigned long)(benchUnlockRemainingMs() / 1000));
  dlog("attitude R %.1f P %.1f  gyro %.1f %.1f %.1f\n", t.roll, t.pitch, t.gx, t.gy, t.gz);
  dlog("loop %.0f Hz, compute %lu us, IMU errors %lu, stack headroom %lu B\n", t.loopHz,
       (unsigned long)t.workUs, (unsigned long)t.imuErrors, (unsigned long)t.stackFree);
  dlog("control %s, age %lu ms, packets %lu, throttle %.2f, bad app packets %lu\n",
       rcSourceName((RcSource)t.rcSource), (unsigned long)(t.rxAge > 99999 ? 99999 : t.rxAge),
       (unsigned long)t.rxPackets, t.throttle, (unsigned long)appLinkBadPackets());
  dlog("battery %.2f V\n", batteryVoltage());
  dlog("blackbox %s%s%s\n", blackboxFrozen() ? "frozen: " : "idle", blackboxFrozen() ? blackboxReason() : "",
       blackboxSavedExists() ? "  [saved fault log in flash: bb saved]" : "");
  dlog("Wi-Fi '%s' pass '%s', IP %s, %d client(s)\n", netSsid().c_str(), WIFI_AP_PASS,
       netIp().toString().c_str(), netClients());
}

bool is(const char *a, const char *b) { return a && strcmp(a, b) == 0; }

void request(BenchJob j, int a, bool armed) {
  String err;
  if (!benchRequest(j, a, armed, err)) dlog("! %s\n", err.c_str());
}
}  // namespace

TelemMode cmdTelemMode() { return mode; }

void cmdExecute(const String &lineIn, bool armed) {
  String line = lineIn;
  line.trim();
  if (!line.length()) return;
  line.toLowerCase();
  dlog("> %s\n", line.c_str());

  char buf[96];
  line.toCharArray(buf, sizeof(buf));
  char *argv[4] = {nullptr, nullptr, nullptr, nullptr};
  int argc = 0;
  char *save = nullptr;
  for (char *tok = strtok_r(buf, " \t", &save); tok && argc < 4; tok = strtok_r(nullptr, " \t", &save))
    argv[argc++] = tok;
  const char *c = argv[0];

  // --- always allowed ---
  if (is(c, "help") || is(c, "h") || is(c, "?")) { help(); return; }
  if (is(c, "status")) { status(); return; }
  if (is(c, "stop") || is(c, "x")) { benchAbort(); return; }
  if (is(c, "tel") || is(c, "s")) {
    mode = (mode == TelemMode::Text) ? TelemMode::Off : TelemMode::Text;
    return;
  }
  if (is(c, "plot")) {
    if (is(argv[1], "roll")) mode = TelemMode::PlotRoll;
    else if (is(argv[1], "pitch")) mode = TelemMode::PlotPitch;
    else if (is(argv[1], "yaw")) mode = TelemMode::PlotYaw;
    else if (is(argv[1], "att")) mode = TelemMode::Attitude;
    else mode = TelemMode::Off;
    return;
  }
  if (is(c, "get")) { dlog("%s", settingsList().c_str()); return; }
  if (is(c, "lock")) { benchLockMotors(); dlog("motor tests locked\n"); return; }
  if (is(c, "bb")) {
    if (is(argv[1], "clear")) { dlog(blackboxClearSaved() ? "saved fault log deleted\n" : "! nothing to delete\n"); return; }
    bool saved = is(argv[1], "saved");
    // Raw to Serial, not dlog: a dump is far bigger than the log ring.
    bool ok = blackboxDump(saved, [](const char *d, size_t n, void *) { Serial.write((const uint8_t *)d, n); }, nullptr);
    if (!ok) dlog(saved ? "! no saved fault log\n" : "! nothing recorded yet (arm, fly, disarm, wait 1 s)\n");
    return;
  }

  // --- disarmed only ---
  if (armed) { dlog("! refused while ARMED (only status/tel/plot/get/stop)\n"); return; }

  if (is(c, "props")) {
    if (is(argv[1], "off")) {
      benchUnlockMotors();
      dlog("motor tests UNLOCKED for %d min. Props must really be off!\n", PROPS_OFF_UNLOCK_MS / 60000);
    } else {
      dlog("! usage: props off\n");
    }
  } else if (is(c, "cal") || is(c, "c")) {
    request(BenchJob::Calibrate, 0, armed);
  } else if (is(c, "selftest")) {
    request(BenchJob::SelfTest, is(argv[1], "motors") ? 1 : 0, armed);
  } else if (is(c, "orient")) {
    request(BenchJob::Orient, 0, armed);
  } else if (is(c, "i2c")) {
    request(BenchJob::I2cScan, 0, armed);
  } else if ((c[0] == 't' || c[0] == 'm') && c[1] >= '1' && c[1] <= '4' && c[2] == 0) {
    request(BenchJob::MotorOne, c[1] - '1', armed);
  } else if (is(c, "mall")) {
    request(BenchJob::MotorSeq, 0, armed);
  } else if (is(c, "ramp")) {
    int idx = -1;
    if (argv[1] && argv[1][0] >= '1' && argv[1][0] <= '4') idx = argv[1][0] - '1';
    else if (argv[1] && !is(argv[1], "all")) { dlog("! usage: ramp [1-4|all]\n"); return; }
    request(BenchJob::MotorRamp, idx, armed);
  } else if (is(c, "mix")) {
    int ax = is(argv[1], "roll") ? 0 : is(argv[1], "pitch") ? 1 : is(argv[1], "yaw") ? 2 : -1;
    if (ax < 0) { dlog("! usage: mix roll|pitch|yaw\n"); return; }
    request(BenchJob::MixTest, ax, armed);
  } else if (is(c, "set")) {
    if (argc < 3) { dlog("! usage: set <key> <value>\n"); return; }
    char *end = nullptr;
    float v = strtof(argv[2], &end);
    if (end == argv[2] || *end) { dlog("! not a number: %s\n", argv[2]); return; }
    String err;
    if (settingsSet(argv[1], v, err)) dlog("%s = %.6g (not saved yet: 'save')\n", argv[1], v);
    else dlog("! %s\n", err.c_str());
  } else if (is(c, "save")) {
    dlog(settingsSave() ? "settings saved to flash\n" : "! save failed\n");
  } else if (is(c, "load")) {
    dlog(settingsLoad() ? "settings loaded from flash\n" : "! nothing valid stored\n");
  } else if (is(c, "defaults")) {
    settingsDefaults();
    dlog("defaults restored (not saved yet: 'save')\n");
  } else if (is(c, "reboot")) {
    dlog("rebooting...\n");
    delay(100);
    ESP.restart();
  } else {
    dlog("! unknown command '%s' (help)\n", c);
  }
}
