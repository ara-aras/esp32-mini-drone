#include "bench.h"
#include "config.h"
#include "logbuf.h"
#include "ahrs.h"
#include "mixer.h"
#include "pid.h"
#include "radio.h"
#include "battery.h"
#include <Wire.h>
#include <math.h>

namespace {
// ---------- request channel (loop task -> flight task) ----------
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
BenchJob pendingJob = BenchJob::None;
int pendingArg = 0;
bool abortReq = false;
volatile bool busy = false;
volatile uint32_t unlockUntil = 0;

bool unlocked() { return unlockUntil != 0 && millis() < unlockUntil; }

bool needsMotors(BenchJob j, int arg) {
  return j == BenchJob::MotorOne || j == BenchJob::MotorSeq ||
         j == BenchJob::MotorRamp || j == BenchJob::MixTest ||
         (j == BenchJob::SelfTest && arg == 1);
}

// ---------- flight-task state ----------
BenchJob job = BenchJob::None;
int arg = 0;
int phase = 0;
int announced = -1;
int evalCount = 0;
uint32_t phaseStart = 0;
uint32_t levelSince = 0;
float yawAcc = 0;
int nPass = 0, nWarn = 0, nFail = 0;
float baseVib = 0, baseV = 0;

const char *MOTOR_NAMES[4] = {"M1 front-left (CW)", "M2 front-right (CCW)",
                              "M3 rear-right (CW)", "M4 rear-left (CCW)"};
const char *AXIS_NAMES[3] = {"ROLL", "PITCH", "YAW"};

struct Stats {
  double n, sgx, sgy, sgz, qgx, qgy, qgz, sax, say, saz, qax, qay, qaz;
  float vmin;
  void reset() { *this = Stats{}; vmin = 99; }
  void add(const ImuSample &s, float v) {
    n++;
    sgx += s.gx; sgy += s.gy; sgz += s.gz;
    qgx += s.gx * s.gx; qgy += s.gy * s.gy; qgz += s.gz * s.gz;
    sax += s.ax; say += s.ay; saz += s.az;
    qax += s.ax * s.ax; qay += s.ay * s.ay; qaz += s.az * s.az;
    if (v > 0.5f && v < vmin) vmin = v;
  }
  static double var(double q, double s, double n) { return n ? fmax(0.0, q / n - (s / n) * (s / n)) : 0; }
  float gyroStd() const {
    return sqrt(fmax(var(qgx, sgx, n), fmax(var(qgy, sgy, n), var(qgz, sgz, n))));
  }
  float gyroMeanMax() const {
    return n ? fmax(fabs(sgx / n), fmax(fabs(sgy / n), fabs(sgz / n))) : 0;
  }
  float vib() const { return sqrt(var(qax, sax, n) + var(qay, say, n) + var(qaz, saz, n)); }
  float accNorm() const {
    if (!n) return 0;
    double x = sax / n, y = say / n, z = saz / n;
    return sqrt(x * x + y * y + z * z);
  }
} stats;

void pass(const char *msg) { nPass++; dlog("  [PASS] %s\n", msg); }
void warn(const char *msg) { nWarn++; dlog("  [WARN] %s\n", msg); }
void fail(const char *msg) { nFail++; dlog("  [FAIL] %s\n", msg); }

void start(BenchJob j, int a, uint32_t now) {
  job = j; arg = a; phase = 0; announced = -1; evalCount = 0;
  phaseStart = now; levelSince = 0; yawAcc = 0;
  nPass = nWarn = nFail = 0;
  stats.reset();
}

void finish() {
  if (job == BenchJob::SelfTest || job == BenchJob::Orient) {
    const char *verdict = nFail ? "NOT READY - fix FAIL items"
                        : nWarn ? "OK with warnings - read them"
                                : "ALL GOOD";
    dlog("== RESULT: %d pass, %d warn, %d fail -> %s ==\n", nPass, nWarn, nFail, verdict);
  } else if (job != BenchJob::Calibrate && job != BenchJob::I2cScan) {
    dlog("[bench] done\n");
  }
  job = BenchJob::None;
  busy = false;
}

void nextPhase(uint32_t now) { phase++; phaseStart = now; announced = -1; levelSince = 0; yawAcc = 0; }

// ---------- self-test: static checks ----------
void selfTestStatic(BenchCtx &c) {
  char m[128];
  dlog("== SELF-TEST%s ==\n", arg ? " (with motors)" : "");

  imuIsCalibrated() ? pass("gyro calibrated") : fail("gyro not calibrated (send: cal)");

  snprintf(m, sizeof(m), "loop rate %.0f Hz (target %d)", c.loopHz, LOOP_HZ);
  c.loopHz >= 0.9f * LOOP_HZ ? pass(m) : warn(m);

  uint32_t budget = 1000000UL / LOOP_HZ;
  snprintf(m, sizeof(m), "loop compute %lu us of %lu us budget", (unsigned long)c.workUs, (unsigned long)budget);
  c.workUs < budget / 2 ? pass(m) : warn(m);

  snprintf(m, sizeof(m), "IMU read errors since boot: %lu", (unsigned long)c.imuErrors);
  c.imuErrors == 0 ? pass(m) : warn(m);

#if VBAT_SENSE_ENABLED
  float v = batteryVoltage();
  snprintf(m, sizeof(m), "battery %.2f V (arm needs >= %.2f)", v, VBAT_MIN_ARM);
  if (v < 1.0f) warn("battery reads ~0 V: on USB only, or divider not wired");
  else if (v >= VBAT_MIN_ARM) pass(m);
  else fail(m);
#else
  warn("battery sensing disabled in config.h");
#endif

  RcInput rc;
  radioGet(rc);
  if (rc.valid && rc.ageMs < FAILSAFE_TIMEOUT_MS) {
    snprintf(m, sizeof(m), "control link OK via %s (%lu packets, age %lu ms)",
             rcSourceName(rc.source), (unsigned long)rc.packets, (unsigned long)rc.ageMs);
    pass(m);
    snprintf(m, sizeof(m), "sticks thr %.2f roll %.2f pitch %.2f yaw %.2f arm %d (should be centred/zero)",
             rc.throttle, rc.roll, rc.pitch, rc.yaw, rc.armSwitch);
    (rc.throttle <= ARM_MAX_THROTTLE && !rc.armSwitch) ? pass(m) : warn(m);
  } else if (rc.valid) {
    warn("control link stale - app closed / transmitter off / out of range?");
  } else {
    warn("no controller yet: connect the ESP-Drone app or power the transmitter");
  }

  snprintf(m, sizeof(m), "free heap %lu bytes", (unsigned long)ESP.getFreeHeap());
  ESP.getFreeHeap() > ST_MIN_FREE_HEAP ? pass(m) : warn(m);

  snprintf(m, sizeof(m), "flight task stack headroom %lu bytes", (unsigned long)c.stackFree);
  c.stackFree >= ST_MIN_FLIGHT_STACK ? pass(m) : warn(m);

  // Mixer logic: each axis must speed up the correct motors.
  float o[4];
  mixQuadX(0.5f, 0.1f, 0, 0, o, 0);
  bool rOk = o[0] > 0.5f && o[3] > 0.5f && o[1] < 0.5f && o[2] < 0.5f;
  mixQuadX(0.5f, 0, 0.1f, 0, o, 0);
  bool pOk = o[0] > 0.5f && o[1] > 0.5f && o[2] < 0.5f && o[3] < 0.5f;
  mixQuadX(0.5f, 0, 0, 0.1f, o, 0);
  bool yOk = o[1] > 0.5f && o[3] > 0.5f && o[0] < 0.5f && o[2] < 0.5f;
  (rOk && pOk && yOk) ? pass("mixer directions (roll/pitch/yaw)") : fail("mixer direction logic broken");

  // PID logic: positive error -> positive output, output limited.
  Pid p(c.st.rpKp, c.st.rpKi, c.st.rpKd, PID_I_LIMIT, PID_OUT_LIMIT, DTERM_LPF_HZ);
  float a = p.update(50, 0, 0.002f, false);
  float b = p.update(1e6f, 0, 0.002f, false);
  (a > 0 && b <= PID_OUT_LIMIT + 1e-6f) ? pass("PID sign and output limit")
                                         : fail("PID sign/limit wrong (gains zero?)");

  snprintf(m, sizeof(m), "failsafe timeout %d ms", FAILSAFE_TIMEOUT_MS);
  FAILSAFE_TIMEOUT_MS <= 1000 ? pass(m) : warn(m);

  dlog("Measuring sensors for 1.5 s - keep the drone STILL and LEVEL...\n");
}

void selfTestSensors(BenchCtx &c) {
  char m[128];
  float gs = stats.gyroStd();
  snprintf(m, sizeof(m), "gyro noise %.2f deg/s", gs);
  if (gs < ST_GYRO_NOISE_OK_DPS) pass(m);
  else if (gs < ST_GYRO_NOISE_BAD_DPS) warn(m);
  else fail(m);

  float gm = stats.gyroMeanMax();
  snprintf(m, sizeof(m), "gyro residual bias %.2f deg/s", gm);
  gm < ST_GYRO_BIAS_OK_DPS ? pass(m) : warn("gyro drifted since calibration (send: cal)");

  float an = stats.accNorm();
  snprintf(m, sizeof(m), "accel magnitude %.3f g (expect ~1.000)", an);
  (an > 0.9f && an < 1.1f) ? pass(m) : fail(m);

  snprintf(m, sizeof(m), "attitude roll %.1f pitch %.1f deg", c.roll, c.pitch);
  (fabs(c.roll) < ST_LEVEL_OK_DEG && fabs(c.pitch) < ST_LEVEL_OK_DEG)
      ? pass(m) : warn("not level: level the drone, or set trim_roll/trim_pitch");

  baseVib = stats.vib();
  baseV = batteryVoltage();
  dlog("  [INFO] baseline vibration %.4f g\n", baseVib);
}

bool motorEvidence(int k, char *m, size_t len) {
  float vib = stats.vib();
  float sag = (baseV > 1.0f && stats.vmin < 90) ? baseV - stats.vmin : 0;
  bool vibUp = vib > baseVib * ST_VIB_FACTOR + ST_VIB_OFFSET_G;
  bool sagged = sag > ST_SAG_MIN_V;
  snprintf(m, len, "%s: vibration %.4f g (base %.4f), battery sag %.2f V", MOTOR_NAMES[k], vib, baseVib, sag);
  return vibUp || sagged;
}
}  // namespace

// ================= loop-task API =================
bool benchRequest(BenchJob j, int a, bool armed, String &err) {
  if (armed) { err = "disarm first"; return false; }
  if (busy) { err = "a bench test is running - send 'stop' first"; return false; }
  if (needsMotors(j, a) && !unlocked()) {
    err = "motor tests are locked. REMOVE PROPELLERS, then send 'props off'";
    return false;
  }
  portENTER_CRITICAL(&mux);
  pendingJob = j;
  pendingArg = a;
  busy = true;
  portEXIT_CRITICAL(&mux);
  return true;
}

void benchAbort() {
  portENTER_CRITICAL(&mux);
  abortReq = true;
  pendingJob = BenchJob::None;
  portEXIT_CRITICAL(&mux);
}

bool benchBusy() { return busy; }
void benchUnlockMotors() { unlockUntil = millis() + PROPS_OFF_UNLOCK_MS; if (!unlockUntil) unlockUntil = 1; }
void benchLockMotors() { unlockUntil = 0; }
uint32_t benchUnlockRemainingMs() {
  uint32_t u = unlockUntil, now = millis();
  return (u && now < u) ? u - now : 0;
}

// ================= flight-task side =================
void benchDiscardPending() {
  bool dropped = false;
  portENTER_CRITICAL(&mux);
  if (job == BenchJob::None && pendingJob != BenchJob::None) {
    pendingJob = BenchJob::None;
    busy = false;
    dropped = true;
  }
  portEXIT_CRITICAL(&mux);
  if (dropped) dlog("[bench] request dropped: drone armed\n");
}

bool benchTick(BenchCtx &c, float out[4]) {
  for (int i = 0; i < 4; i++) out[i] = 0;

  portENTER_CRITICAL(&mux);
  bool ab = abortReq;
  abortReq = false;
  BenchJob nj = pendingJob;
  int na = pendingArg;
  pendingJob = BenchJob::None;
  portEXIT_CRITICAL(&mux);

  uint32_t now = millis();
  if (ab) {
    if (job != BenchJob::None) dlog("[bench] STOPPED\n");
    job = BenchJob::None;
    busy = false;
    return false;
  }
  if (nj != BenchJob::None && job == BenchJob::None) start(nj, na, now);
  if (job == BenchJob::None) return false;

  uint32_t el = now - phaseStart;
  float lvl = c.st.testLevel;
  char m[128];

  switch (job) {
    case BenchJob::Calibrate: {
      dlog("[bench] calibrating gyro - keep still...\n");
      if (imuCalibrateGyro()) { ahrsReset(c.s); dlog("[bench] calibration OK\n"); }
      c.timingReset = true;
      finish();
      return false;
    }

    case BenchJob::I2cScan: {
      dlog("[bench] I2C scan:\n");
      int found = 0;
      for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
          dlog("  device at 0x%02X%s\n", a, a == IMU_ADDR ? "  <- IMU" : "");
          found++;
        }
      }
      dlog("[bench] %d device(s) found\n", found);
      c.timingReset = true;
      finish();
      return false;
    }

    case BenchJob::MotorOne:
      if (el == 0 || announced < 0) {
        announced = 0;
        dlog("[bench] spinning %s at %.0f%%\n", MOTOR_NAMES[arg], lvl * 100);
      }
      if (el >= MOTOR_TEST_MS) { finish(); return false; }
      out[arg] = lvl;
      return true;

    case BenchJob::MotorSeq: {
      const uint32_t on = 1200, cycle = 1600;
      int k = el / cycle;
      if (k >= 4) { finish(); return false; }
      if (announced != k) {
        announced = k;
        dlog("[bench] %s - check position and spin direction\n", MOTOR_NAMES[k]);
      }
      if (el % cycle < on) out[k] = lvl;
      return true;
    }

    case BenchJob::MotorRamp: {
      const uint32_t T = 4000;
      if (announced < 0) {
        announced = 0;
        dlog("[bench] ramp %s up and down\n", arg < 0 ? "ALL motors" : MOTOR_NAMES[arg]);
      }
      if (el >= T) { finish(); return false; }
      float x = (float)el / T;
      float peak = min(2.0f * lvl, 0.35f);
      float v = peak * (x < 0.5f ? 2 * x : 2 - 2 * x);
      for (int i = 0; i < 4; i++) if (arg < 0 || arg == i) out[i] = v;
      return true;
    }

    case BenchJob::MixTest: {
      static const char *EXPECT[3][2] = {
          {"M1 + M4 (LEFT side) faster  -> would roll RIGHT",
           "M2 + M3 (RIGHT side) faster -> would roll LEFT"},
          {"M1 + M2 (FRONT) faster      -> would pitch NOSE UP",
           "M3 + M4 (REAR) faster       -> would pitch NOSE DOWN"},
          {"M2 + M4 (CCW props) faster  -> would yaw nose RIGHT",
           "M1 + M3 (CW props) faster   -> would yaw nose LEFT"}};
      int half = el < 2500 ? 0 : 1;
      if (el >= 5000) { finish(); return false; }
      if (announced != half) {
        announced = half;
        dlog("[bench] %s %s: expect %s\n", AXIS_NAMES[arg], half ? "-" : "+", EXPECT[arg][half]);
      }
      float d = half ? -MIX_TEST_DELTA : MIX_TEST_DELTA;
      mixQuadX(lvl, arg == 0 ? d : 0, arg == 1 ? d : 0, arg == 2 ? d : 0, out, 0);
      return true;
    }

    case BenchJob::Orient: {
      static const char *PROMPT[] = {
          "Hold the drone LEVEL",
          "Tilt RIGHT side DOWN (>25 deg)",
          "Back to LEVEL",
          "Tilt NOSE UP (>25 deg)",
          "Back to LEVEL",
          "Rotate nose to the RIGHT (clockwise from above, >60 deg)",
      };
      if (phase >= 6) { finish(); return false; }
      if (announced != phase) {
        if (phase == 0) dlog("== ORIENTATION TEST (%d s per step) ==\n", ORIENT_STEP_TIMEOUT_MS / 1000);
        announced = phase;
        dlog("  -> %s\n", PROMPT[phase]);
      }
      if (el > ORIENT_STEP_TIMEOUT_MS) {
        snprintf(m, sizeof(m), "timeout: %s", PROMPT[phase]);
        fail(m);
        nextPhase(now);
        return false;
      }
      switch (phase) {
        case 0: case 2: case 4:
          if (fabs(c.roll) < 8 && fabs(c.pitch) < 8) {
            if (!levelSince) levelSince = now;
            if (now - levelSince > 1000) {
              if (phase == 0) pass("level detected");
              nextPhase(now);
            }
          } else {
            levelSince = 0;
          }
          break;
        case 1:
          if (c.roll > 25) { pass("roll sign correct"); nextPhase(now); }
          else if (c.roll < -25) { fail("roll reads NEGATIVE: fix IMU_MAP (by sign)"); nextPhase(now); }
          else if (fabs(c.pitch) > 25) { fail("tilt showed up on PITCH: X/Y axes swapped in IMU_MAP"); nextPhase(now); }
          break;
        case 3:
          if (c.pitch > 25) { pass("pitch sign correct"); nextPhase(now); }
          else if (c.pitch < -25) { fail("pitch reads NEGATIVE: fix IMU_MAP (bx sign)"); nextPhase(now); }
          else if (fabs(c.roll) > 25) { fail("tilt showed up on ROLL: X/Y axes swapped in IMU_MAP"); nextPhase(now); }
          break;
        case 5:
          yawAcc += c.s.gz * c.dt;
          if (yawAcc > 60) { pass("yaw sign correct"); nextPhase(now); }
          else if (yawAcc < -60) { fail("yaw rate NEGATIVE: fix IMU_MAP (bz sign)"); nextPhase(now); }
          break;
      }
      return false;
    }

    case BenchJob::SelfTest: {
      if (phase == 0) {
        selfTestStatic(c);
        nextPhase(now);
        return false;
      }
      if (phase == 1) {
        stats.add(c.s, batteryVoltage());
        if (el < 1500) return false;
        selfTestSensors(c);
        if (arg == 1) {
          if (!unlocked()) { warn("motor check skipped: motor tests locked"); finish(); return false; }
          dlog("Motor check: each motor spins 1.5 s. PROPS MUST BE OFF.\n");
          stats.reset();
          nextPhase(now);
          return false;
        }
        finish();
        return false;
      }
      // phase 2: motor spin checks
      const uint32_t spin = 1500, cycle = 2200;
      int k = el / cycle;
      uint32_t w = el % cycle;
      if (k >= 4) {
        dlog("  [INFO] vibration/sag can't prove direction - use 'mall' and look.\n");
        finish();
        return false;
      }
      if (announced != k) { announced = k; dlog("  spinning %s...\n", MOTOR_NAMES[k]); }
      if (w < spin) {
        out[k] = lvl;
        if (w >= 500) stats.add(c.s, batteryVoltage());
        return true;
      }
      if (evalCount == k) {
        bool ok = motorEvidence(k, m, sizeof(m));
        if (ok) {
          pass(m);
        } else {
          char w2[200];
          snprintf(w2, sizeof(w2), "%s -> no response detected: check motor/MOSFET/wiring", m);
          warn(w2);
        }
        evalCount++;
        stats.reset();
      }
      return false;
    }

    default:
      finish();
      return false;
  }
}
