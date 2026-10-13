// ESP32 quadcopter flight controller (Arduino IDE)
// Angle mode roll/pitch, rate mode yaw, ESP-NOW control,
// bench test suite, live tuning, and a Wi-Fi web dashboard.
//
// SAFETY: remove propellers for every bench test. Read README.md first.
// Fly with the ESP-Drone phone app (join Wi-Fi ESP-DRONE_xxxx, password from config.h)
// or the ESP-NOW Transmitter sketch. Type `help` in Serial Monitor
// (115200, Newline) or open http://192.168.43.42 for the test dashboard.

#include "src/config.h"
#include "src/imu.h"
#include "src/ahrs.h"
#include "src/pid.h"
#include "src/mixer.h"
#include "src/motors.h"
#include "src/radio.h"
#include "src/battery.h"
#include "src/settings.h"
#include "src/telemetry.h"
#include "src/bench.h"
#include "src/commands.h"
#include "src/logbuf.h"
#include "src/web.h"
#include "src/net.h"
#include "src/applink.h"
#include "src/blackbox.h"
#include <esp_timer.h>
#include <Wire.h>

Pid rollRate (RATE_RP_KP,  RATE_RP_KI,  RATE_RP_KD,  PID_I_LIMIT, PID_OUT_LIMIT, DTERM_LPF_HZ);
Pid pitchRate(RATE_RP_KP,  RATE_RP_KI,  RATE_RP_KD,  PID_I_LIMIT, PID_OUT_LIMIT, DTERM_LPF_HZ);
Pid yawRate  (RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD, PID_I_LIMIT, PID_OUT_LIMIT, DTERM_LPF_HZ);

void applySettings(const Settings &st) {
  rollRate.setGains(st.rpKp, st.rpKi, st.rpKd);
  pitchRate.setGains(st.rpKp, st.rpKi, st.rpKd);
  yawRate.setGains(st.yawKp, st.yawKi, st.yawKd);
  ahrsSetTrim(st.trimRoll, st.trimPitch);
}

// State changes go to Serial, the web log AND the app's console view.
void stateMsg(const char *prefix, const char *why) {
  dlog("[state] %s%s\n", prefix, why);
  char line[64];
  snprintf(line, sizeof(line), "%s%s\n", prefix, why);
  appLinkConsole(line);
}

void resetPids() {
  rollRate.reset();
  pitchRate.reset();
  yawRate.reset();
}

// Set by loop() when the flight task stops publishing telemetry while armed
// (I2C hang, crash in the task...). loop() cuts the motors itself; the flight
// task disarms when it comes back so it cannot re-drive them.
volatile bool flightStalled = false;

// ================= flight task (core 1, LOOP_HZ) =================
void flightTask(void *) {
  TickType_t period = pdMS_TO_TICKS(1000 / LOOP_HZ);
  if (period == 0) period = 1;
  TickType_t lastWake = xTaskGetTickCount();
  int64_t lastUs = esp_timer_get_time();

  Settings st = settingsGet();
  uint32_t stVer = settingsVersion();
  applySettings(st);

  bool armed = false;
  bool switchWasOff = false;   // arm switch must be seen OFF (over a live link) before arming
  RcSource armedSource = RcSource::None;  // control source latched at arming
  uint32_t armedAtMs = 0;
  int imuFailStreak = 0;
  int loopCount = 0;
  uint32_t stackFree = 0;
  bool linkWasOk = false;
  RcSource lastSource = RcSource::None;
  uint32_t imuErrors = 0;
  float out[4] = {0, 0, 0, 0};
  float hzFilt = LOOP_HZ;
  float workPeak = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    int64_t nowUs = esp_timer_get_time();
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;
    if (dt <= 0 || dt > 0.05f) dt = 1.0f / LOOP_HZ;
    hzFilt += 0.01f * (1.0f / dt - hzFilt);
    int64_t workStart = nowUs;  // compute time includes the I2C read

    uint32_t v = settingsVersion();
    if (v != stVer) {
      stVer = v;
      st = settingsGet();
      applySettings(st);
    }

    // --- sensors ---
    if (flightStalled) {
      if (armed) {
        armed = false;
        stateMsg("DISARMED: ", "flight loop stalled (watchdog)");
        blackboxDisarm("flight loop stalled", true);
      }
      flightStalled = false;
    }

    ImuSample s;
    if (!imuRead(s)) {
      imuErrors++;
      if (++imuFailStreak >= IMU_FAIL_DISARM_STREAK && armed) {
        armed = false;
        stateMsg("DISARMED: ", "IMU read failure");
        blackboxDisarm("IMU read failure", true);
      }
      if (!armed) motorsStop();
      continue;
    }
    imuFailStreak = 0;
    ahrsUpdate(s, dt);
    float roll = ahrsRoll(), pitch = ahrsPitch();

    RcInput rc;
    radioGet(rc);
    bool linkOk = rc.valid && rc.ageMs < FAILSAFE_TIMEOUT_MS;

    // Link / source transitions are logged even while disarmed: a flaky link
    // on the bench is the same flaky link that fails in the air.
    if (linkOk != linkWasOk) {
      linkWasOk = linkOk;
      dlog("[link] %s (%s)\n", linkOk ? "OK" : "LOST", rcSourceName(rc.source));
    }
    if (rc.source != lastSource) {
      lastSource = rc.source;
      dlog("[link] control source: %s\n", rcSourceName(rc.source));
    }

    // --- arming state machine ---
    // The switch must be seen OFF over a live link: a link dropout does not
    // count, so a failsafe can never be followed by a silent re-arm.
    if (linkOk && !rc.armSwitch) switchWasOff = true;

    if (armed) {
      const char *why = nullptr;
      bool fault = true;  // everything except a pilot-commanded disarm
      if (!linkOk) why = "FAILSAFE: link lost";
      else if (rc.source != armedSource) why = "control source changed";
      else if (!rc.armSwitch) { why = rc.source == RcSource::App ? "throttle at zero" : "arm switch off"; fault = false; }
      else if (fabs(roll) > CRASH_TILT_DEG || fabs(pitch) > CRASH_TILT_DEG) why = "crash tilt";
      if (why) {
        armed = false;
        switchWasOff = false;  // any disarm requires a fresh switch cycle
        stateMsg("DISARMED: ", why);
        blackboxDisarm(why, fault);  // faults are also saved to flash
      }
    } else if (linkOk && rc.armSwitch && switchWasOff) {
      const char *why = nullptr;
      if (benchBusy())                               why = "bench test running";
      else if (!imuIsCalibrated())                   why = "gyro not calibrated";
      else if (!rc.armImpliesLowThrottle &&
               rc.throttle > ARM_MAX_THROTTLE)       why = "throttle not low";
      else if (!batteryOkToArm())                    why = "battery too low";
      else if (fabs(roll) > ARM_MAX_TILT_DEG ||
               fabs(pitch) > ARM_MAX_TILT_DEG)       why = "not level";
      switchWasOff = false;  // must cycle the switch to try again
      if (why) {
        stateMsg("ARM REFUSED: ", why);
      } else {
        armed = true;
        armedSource = rc.source;
        armedAtMs = millis();
        benchDiscardPending();
        resetPids();
        blackboxArm();
        stateMsg("ARMED via ", rcSourceName(rc.source));
      }
    }

    float sp[3] = {0, 0, 0}, pidOut[3] = {0, 0, 0};
    float thrUsed = 0;
    bool blocked = false;

    if (!armed) {
      // --- disarmed: bench tests own the motors ---
      resetPids();
      BenchCtx ctx{s, roll, pitch, dt, hzFilt, imuErrors, (uint32_t)workPeak, stackFree, st, false};
      bool active = benchTick(ctx, out);
      if (active) motorsWrite(out);
      else {
        motorsStop();
        for (int i = 0; i < 4; i++) out[i] = 0;
      }
      if (ctx.timingReset) {
        blocked = true;
        lastUs = esp_timer_get_time();
        lastWake = xTaskGetTickCount();
      }
    } else {
      // --- control ---
      float tgtRoll  =  rc.roll  * st.maxAngle;
      float tgtPitch = -rc.pitch * st.maxAngle;  // stick forward = nose down

      sp[0] = constrain(st.angleKp * (tgtRoll  - roll),  -st.maxRateRp, st.maxRateRp);
      sp[1] = constrain(st.angleKp * (tgtPitch - pitch), -st.maxRateRp, st.maxRateRp);
      sp[2] = rc.yaw * st.maxRateYaw;

      // Soft start: the app arms on the first non-zero thrust packet, which a
      // fast stick flick can put well above zero. Cap throttle for a moment.
      float cap = (float)(millis() - armedAtMs) / ARM_THROTTLE_RAMP_MS;
      float throttle = rc.throttle;
      if (cap < 1.0f && throttle > cap) throttle = cap;
      thrUsed = throttle;

      bool integrate = throttle > I_TERM_MIN_THROTTLE;
      pidOut[0] = rollRate.update (sp[0], s.gx, dt, integrate);
      pidOut[1] = pitchRate.update(sp[1], s.gy, dt, integrate);
      pidOut[2] = yawRate.update  (sp[2], s.gz, dt, integrate);

      float thr = st.motorIdle + throttle * (MOTOR_MAX - st.motorIdle);
      mixQuadX(thr, pidOut[0], pidOut[1], pidOut[2], out, st.motorIdle);
      motorsWrite(out);
    }

    if (!blocked) {
      float work = (float)(esp_timer_get_time() - workStart);
      workPeak = max(workPeak * 0.999f, work);
    }

    // --- publish telemetry ---
    if (++loopCount >= LOOP_HZ) {  // once a second: walks the stack, keep it rare
      loopCount = 0;
      stackFree = uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
    }
    Telemetry t;
    t.roll = roll; t.pitch = pitch;
    t.gx = s.gx; t.gy = s.gy; t.gz = s.gz;
    t.accNorm = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
    for (int i = 0; i < 3; i++) { t.sp[i] = sp[i]; t.pid[i] = pidOut[i]; }
    for (int i = 0; i < 4; i++) t.motor[i] = out[i];
    t.throttle = rc.throttle;
    t.armed = armed;
    t.benchBusy = benchBusy();
    t.rxAge = rc.ageMs;
    t.rxPackets = rc.packets;
    t.imuErrors = imuErrors;
    t.loopHz = hzFilt;
    t.workUs = (uint32_t)workPeak;
    t.stackFree = stackFree;
    t.stampMs = millis();
    t.rcSource = (uint8_t)rc.source;
    telemetryPublish(t);

    if (blackboxRecording()) {
      BbFrame f;
      f.ms = t.stampMs;
      f.roll = (int16_t)(roll * 100); f.pitch = (int16_t)(pitch * 100);
      f.gx = (int16_t)constrain(s.gx * 10, -32767.f, 32767.f);
      f.gy = (int16_t)constrain(s.gy * 10, -32767.f, 32767.f);
      f.gz = (int16_t)constrain(s.gz * 10, -32767.f, 32767.f);
      for (int i = 0; i < 3; i++) {
        f.sp[i] = (int16_t)(sp[i] * 10);
        f.pid[i] = (int16_t)(pidOut[i] * 10000);
      }
      for (int i = 0; i < 4; i++) f.m[i] = (uint8_t)(out[i] * 255);
      f.thr = (uint8_t)(thrUsed * 255);
      f.rcR = (int8_t)(rc.roll * 127); f.rcP = (int8_t)(rc.pitch * 127); f.rcY = (int8_t)(rc.yaw * 127);
      f.vbat = (uint8_t)constrain(batteryVoltage() * 40, 0.f, 255.f);
      f.flags = (armed ? 1 : 0) | (((uint8_t)rc.source & 3) << 1) | (linkOk ? 8 : 0);
      blackboxRecord(f);
    }
  }
}

// ================= helpers =================
void haltWithError(const char *msg) {
  motorsStop();
  dlog("FATAL: %s\n", msg);
  for (;;) {
    logFlushSerial();
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(100);
  }
}

void readSerialCommands(bool armed) {
  static char buf[96];
  static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) {
        buf[len] = 0;
        len = 0;
        cmdExecute(String(buf), armed);
      }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }
}

void printTelemetry(const Telemetry &t) {
  switch (cmdTelemMode()) {
    case TelemMode::Text:
      Serial.printf("%s R%6.1f P%6.1f | g %6.1f %6.1f %6.1f | thr %.2f | M %.2f %.2f %.2f %.2f | "
                    "V %.2f | rx %lums | %.0fHz %luus\n",
                    t.armed ? "ARM" : "dis", t.roll, t.pitch, t.gx, t.gy, t.gz, t.throttle,
                    t.motor[0], t.motor[1], t.motor[2], t.motor[3], batteryVoltage(),
                    (unsigned long)(t.rxAge > 99999 ? 99999 : t.rxAge), t.loopHz,
                    (unsigned long)t.workUs);
      break;
    case TelemMode::PlotRoll:
      Serial.printf("sp:%.1f gyro:%.1f angle:%.1f out_x1000:%.1f\n", t.sp[0], t.gx, t.roll, t.pid[0] * 1000);
      break;
    case TelemMode::PlotPitch:
      Serial.printf("sp:%.1f gyro:%.1f angle:%.1f out_x1000:%.1f\n", t.sp[1], t.gy, t.pitch, t.pid[1] * 1000);
      break;
    case TelemMode::PlotYaw:
      Serial.printf("sp:%.1f gyro:%.1f out_x1000:%.1f\n", t.sp[2], t.gz, t.pid[2] * 1000);
      break;
    case TelemMode::Attitude:  // for tools/gyro3d.py: roll pitch yaw-rate motors armed
      Serial.printf("att %.2f %.2f %.1f %.2f %.2f %.2f %.2f %d %.3f\n", t.roll, t.pitch, t.gz,
                    t.motor[0], t.motor[1], t.motor[2], t.motor[3], t.armed, t.accNorm);
      break;
    default:
      break;
  }
}

// ================= Arduino entry points =================
void setup() {
  // Motors first, so they are held low as early as possible.
  bool motorsOk = motorsBegin();
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  delay(300);
  dlog("\n=== ESP32 Drone FC ===\n");
  if (!motorsOk) dlog("[motors] PWM attach reported an error\n");

  settingsBegin();
  batteryBegin();
  blackboxBegin();
  if (!imuBegin()) {
    // Say what IS on the bus: tells wiring (nothing) from address (0x69) problems.
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { dlog("[i2c] device at 0x%02X\n", a); found++; }
    }
    if (!found) {
      dlog("[i2c] bus empty: check VCC/GND, SDA=%d, SCL=%d\n", PIN_I2C_SDA, PIN_I2C_SCL);
      // Idle line levels: a module with its pull-ups wired reads HIGH on both.
      Wire.end();
      pinMode(PIN_I2C_SDA, INPUT);
      pinMode(PIN_I2C_SCL, INPUT);
      delay(5);
      dlog("[i2c] idle levels (no internal pull-up): SDA=%d SCL=%d  (1/1 = module pull-ups seen, 0 = held low or unwired)\n",
           digitalRead(PIN_I2C_SDA), digitalRead(PIN_I2C_SCL));
      pinMode(PIN_I2C_SDA, INPUT_PULLUP);
      pinMode(PIN_I2C_SCL, INPUT_PULLUP);
      delay(5);
      dlog("[i2c] with internal pull-up: SDA=%d SCL=%d  (0 = shorted to GND / wrong pin)\n",
           digitalRead(PIN_I2C_SDA), digitalRead(PIN_I2C_SCL));
    }
    haltWithError("IMU init failed (try: check wiring, IMU_ADDR)");
  }

  dlog("Calibrating gyro, keep the drone still...\n");
  logFlushSerial();
  while (!imuCalibrateGyro()) {
    dlog("Retrying in 1s...\n");
    logFlushSerial();
    delay(1000);
  }
  ImuSample s;
  if (imuRead(s)) ahrsReset(s);

  netBegin();     // Wi-Fi AP first: app + dashboard + ESP-NOW all share it
  webBegin();
  appLinkBegin();
  if (!radioBegin()) haltWithError("radio init failed");

  xTaskCreatePinnedToCore(flightTask, "flight", 6144, nullptr,
                          FLIGHT_TASK_PRIO, nullptr, FLIGHT_TASK_CORE);
  dlog("Ready. Type 'help'. Start with 'selftest'.\n");
  logFlushSerial();
}

void loop() {
  static uint32_t lastBatt = 0, lastPrint = 0;
  uint32_t now = millis();
  Telemetry t = telemetryGet();

  if (now - lastBatt >= 100) {
    lastBatt = now;
    batteryUpdate();
  }

  // Watchdog: the flight task owns every other safety check, so something
  // outside it must notice when it stops running (I2C hang, task crash).
  static bool wdtReported = false;
  if (t.armed && now - t.stampMs > FLIGHT_WDT_MS) {
    motorsStop();
    flightStalled = true;
    if (!wdtReported) {
      wdtReported = true;
      dlog("[wdt] flight task silent %lu ms while ARMED: motors cut\n", (unsigned long)(now - t.stampMs));
    }
  } else {
    wdtReported = false;
  }

  readSerialCommands(t.armed);
  webLoop();
  appLinkLoop();
  blackboxLoop();
  logFlushSerial();

  static bool wasLow = false;
  if (batteryLow() != wasLow) {
    wasLow = !wasLow;
    dlog("[batt] %.2f V %s\n", batteryVoltage(), wasLow ? "LOW" : "ok");
  }

  // LED: solid = armed, double speed = testing, fast = low batt / no link, slow = ready
  bool linkOk = t.rxAge < FAILSAFE_TIMEOUT_MS;
  if (t.armed) digitalWrite(PIN_LED, HIGH);
  else if (t.benchBusy) digitalWrite(PIN_LED, (now / 250) % 2);
  else if (batteryLow() || !linkOk) digitalWrite(PIN_LED, (now / 100) % 2);
  else digitalWrite(PIN_LED, (now / 500) % 2);

  uint32_t interval = cmdTelemMode() == TelemMode::Text ? 100 : 20;
  if (cmdTelemMode() != TelemMode::Off && now - lastPrint >= interval) {
    lastPrint = now;
    printTelemetry(t);
  }
  delay(2);
}
