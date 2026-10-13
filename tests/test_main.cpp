// Host unit tests for flight logic: mixer, PID, AHRS, stick mapping, and a
// closed-loop single-axis simulation. Run: make   (needs g++ only)
#include <cstdio>
#include <cmath>
#include "../Drone/src/config.h"
#include "../Drone/src/mixer.h"
#include "../Drone/src/pid.h"
#include "../Drone/src/ahrs.h"
#include "../Drone/src/rcmath.h"
#include "../Drone/src/crtp.h"
#include "../Drone/src/apparm.h"
#include <vector>

static int failures = 0, checks = 0;
#define CHECK(cond, msg) do { checks++; if (!(cond)) { failures++; printf("  FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); } } while (0)
#define NEAR(a, b, tol) (std::fabs((a) - (b)) <= (tol))

// ---------------------------------------------------------------- mixer
static void testMixer() {
  puts("[mixer]");
  float o[4];
  mixQuadX(0.5f, 0, 0, 0, o, 0.06f);
  CHECK(o[0] == 0.5f && o[1] == 0.5f && o[2] == 0.5f && o[3] == 0.5f, "pure throttle is equal on all motors");

  mixQuadX(0.5f, 0.1f, 0, 0, o, 0);
  CHECK(o[0] > 0.5f && o[3] > 0.5f && o[1] < 0.5f && o[2] < 0.5f, "roll+ speeds up LEFT motors (M1,M4)");
  mixQuadX(0.5f, 0, 0.1f, 0, o, 0);
  CHECK(o[0] > 0.5f && o[1] > 0.5f && o[2] < 0.5f && o[3] < 0.5f, "pitch+ speeds up FRONT motors (M1,M2)");
  mixQuadX(0.5f, 0, 0, 0.1f, o, 0);
  CHECK(o[1] > 0.5f && o[3] > 0.5f && o[0] < 0.5f && o[2] < 0.5f, "yaw+ speeds up CCW motors (M2,M4)");

  mixQuadX(0.5f, 0.1f, 0.05f, 0.02f, o, 0);
  float sum = o[0] + o[1] + o[2] + o[3];
  CHECK(NEAR(sum, 2.0f, 1e-5f), "corrections do not change total thrust when unsaturated");

  mixQuadX(0.95f, 0.2f, 0, 0, o, 0.06f);
  bool inRange = true;
  for (float v : o) inRange &= (v >= 0.06f && v <= 1.0f);
  CHECK(inRange, "outputs clamped to [idle, 1]");
  CHECK(NEAR(o[0] - o[1], 0.4f, 1e-5f), "high-throttle saturation keeps roll differential (shift down)");
  CHECK(NEAR(o[0], 1.0f, 1e-5f) && NEAR(o[1], 0.6f, 1e-5f), "shift of 0.15 is within MIX_MAX_SHIFT: full differential kept");

  // All three PIDs saturated at full throttle (held-drone case): the old mixer
  // shifted by 0.9 and left three motors at idle. Now the shift is capped and
  // the corrections scaled, so most of the thrust survives.
  mixQuadX(1.0f, 0.3f, 0.3f, 0.3f, o, 0.06f);
  float avg = (o[0] + o[1] + o[2] + o[3]) / 4;
  CHECK(avg >= 1.0f - MIX_MAX_SHIFT - 1e-5f, "saturated corrections cost at most MIX_MAX_SHIFT of thrust");
  CHECK(o[3] > o[2] && o[0] > o[2] && o[1] > o[2], "correction direction survives scaling (M3 lowest for +r+p+y)");
  CHECK(NEAR(o[3], 1.0f, 1e-5f), "fastest motor still at max");

  mixQuadX(0.0f, 0, 0, 0, o, 0.06f);
  CHECK(o[0] == 0.06f && o[2] == 0.06f, "zero throttle sits at idle");

  // Low-side saturation: at idle a correction can only raise motors, never lower
  // them below idle. The differential is halved, not lost (documented behaviour).
  mixQuadX(0.06f, 0.1f, 0, 0, o, 0.06f);
  CHECK(o[1] == 0.06f && o[2] == 0.06f, "idle clamp: slow side pinned at idle");
  CHECK(NEAR(o[0], 0.16f, 1e-5f) && NEAR(o[3], 0.16f, 1e-5f), "idle clamp: fast side still raised");
}

// ---------------------------------------------------------------- PID
static void testPid() {
  puts("[pid]");
  Pid p(0.01f, 0, 0, 0.1f, 0.3f, 40);
  CHECK(p.update(10, 0, 0.002f, false) > 0, "positive error -> positive output");
  CHECK(p.update(-10, 0, 0.002f, false) < 0, "negative error -> negative output");
  CHECK(NEAR(p.update(10, 0, 0.002f, false), 0.1f, 1e-6f), "P term = kp * error");
  CHECK(p.update(1e6f, 0, 0.002f, false) <= 0.3f, "output limited");

  Pid i(0, 1.0f, 0, 0.1f, 1.0f, 40);
  float last = 0;
  for (int k = 0; k < 10; k++) last = i.update(1, 0, 0.002f, true);
  CHECK(NEAR(last, 0.02f, 1e-5f), "I accumulates ki*err*dt");
  for (int k = 0; k < 10000; k++) last = i.update(100, 0, 0.002f, true);
  CHECK(NEAR(last, 0.1f, 1e-6f), "I anti-windup clamps at limit");
  CHECK(i.update(0, 0, 0.002f, false) == 0, "I cleared when integrate=false");

  Pid d(0, 0, 0.001f, 0.1f, 1.0f, 1000);
  d.update(0, 0, 0.002f, false);
  float kick = d.update(500, 0, 0.002f, false);
  CHECK(NEAR(kick, 0, 1e-9f), "no derivative kick on setpoint step (D on measurement)");
  float damp = d.update(0, 10, 0.002f, false);
  CHECK(damp < 0, "D opposes a rising measurement");

  Pid r(0.01f, 1.0f, 0.001f, 0.1f, 1.0f, 40);
  r.update(0, 0, 0.002f, true);
  r.update(100, 0, 0.002f, true);
  r.reset();
  CHECK(NEAR(r.update(0, 0, 0.002f, true), 0, 1e-9f), "reset clears state");
}

// ---------------------------------------------------------------- AHRS
static ImuSample tilted(float rollDeg, float pitchDeg) {
  float r = rollDeg * DEG_TO_RAD, p = pitchDeg * DEG_TO_RAD;
  ImuSample s{};  // specific force of gravity in Z-down body frame
  s.ax = std::sin(p);
  s.ay = -std::cos(p) * std::sin(r);
  s.az = -std::cos(p) * std::cos(r);
  return s;
}

static void testAhrs() {
  puts("[ahrs]");
  ahrsSetTrim(0, 0);
  ahrsReset(tilted(0, 0));
  CHECK(NEAR(ahrsRoll(), 0, 0.01f) && NEAR(ahrsPitch(), 0, 0.01f), "level reads 0/0");
  ahrsReset(tilted(20, 0));
  CHECK(NEAR(ahrsRoll(), 20, 0.1f), "right side down reads roll +20");
  ahrsReset(tilted(0, 15));
  CHECK(NEAR(ahrsPitch(), 15, 0.1f), "nose up reads pitch +15");
  ahrsReset(tilted(-10, -25));
  CHECK(NEAR(ahrsRoll(), -10, 0.2f) && NEAR(ahrsPitch(), -25, 0.2f), "combined negative tilt");

  // gyro integration with accel ignored (norm out of range)
  ahrsReset(tilted(0, 0));
  ImuSample s{};
  s.gx = 90;  // deg/s
  for (int k = 0; k < 500; k++) ahrsUpdate(s, 0.002f);  // 1 s, accel norm 0 -> gyro only
  CHECK(NEAR(ahrsRoll(), 90, 0.5f), "gyro-only integration: 90 deg/s for 1 s = 90 deg");

  // accel correction pulls a wrong estimate back
  ahrsReset(tilted(0, 0));
  for (int k = 0; k < 250; k++) { ImuSample g{}; g.gx = 40; g.az = -1; ahrsUpdate(g, 0.002f); }  // drift to ~20
  ImuSample lvl = tilted(0, 0);
  for (int k = 0; k < 2500; k++) ahrsUpdate(lvl, 0.002f);  // 5 s still and level
  CHECK(std::fabs(ahrsRoll()) < 1.0f, "complementary filter removes drift within 5 s");

  ahrsSetTrim(2, -1);
  ahrsReset(tilted(0, 0));
  CHECK(NEAR(ahrsRoll(), -2, 0.01f) && NEAR(ahrsPitch(), 1, 0.01f), "trim offsets applied");
  ahrsSetTrim(0, 0);
}

// ---------------------------------------------------------------- sticks
static void testSticks() {
  puts("[rc]");
  CHECK(rcStick(0) == 0 && rcStick(STICK_DEADBAND - 1) == 0, "deadband");
  CHECK(NEAR(rcStick(500), 1, 1e-6f) && NEAR(rcStick(-500), -1, 1e-6f), "full deflection = +/-1");
  CHECK(rcStick(2000) == 1.0f, "over-range clamps");
  CHECK(rcStick(260) > 0.4f && rcStick(260) < 0.6f, "mid stick ~0.5");
  CHECK(rcThrottle(-5) == 0 && rcThrottle(1000) == 1 && rcThrottle(1500) == 1, "throttle clamp");
}

// ---------------------------------------------------------------- ESP-Drone app
// Builds a datagram exactly like the Android app: header 0x30, LE floats
// (roll, -pitchStick, yaw), LE uint16 thrust, then an 8-bit sum checksum.
static std::vector<uint8_t> appPacket(float roll, float pitchStick, float yaw, uint16_t thrust) {
  std::vector<uint8_t> b(16);
  b[0] = 0x30;
  float pitch = -pitchStick;  // CommanderPacket.serializeData inverts pitch
  memcpy(&b[1], &roll, 4);
  memcpy(&b[5], &pitch, 4);
  memcpy(&b[9], &yaw, 4);
  memcpy(&b[13], &thrust, 2);
  uint8_t c = 0;
  for (int i = 0; i < 15; i++) c += b[i];
  b[15] = c;
  return b;
}

static void testApp() {
  puts("[esp-drone app]");
  auto pk = appPacket(10, 15, -45, 30000);
  crtp::Frame f{};
  crtp::Commander c{};
  CHECK(crtp::parseUdp(pk.data(), pk.size(), f), "valid app datagram accepted");
  CHECK(f.port == 3 && f.channel == 0 && f.len == 14, "commander port/channel/length");
  CHECK(crtp::parseCommander(f, c), "commander decoded");
  CHECK(c.roll == 10 && c.pitch == -15 && c.yaw == -45 && c.thrust == 30000, "field values");

  RcSticks s = rcFromApp(c.roll, c.pitch, c.yaw, c.thrust);
  CHECK(NEAR(s.roll, 10 / APP_FULL_ANGLE_DEG, 1e-5f), "roll right stick -> roll +");
  CHECK(NEAR(s.pitch, 1.0f, 1e-5f), "full forward stick -> pitch +1 (forward)");
  CHECK(NEAR(s.yaw, -0.5f, 1e-5f), "yaw left half -> -0.5");
  CHECK(NEAR(s.throttle, 0.5f, 1e-5f), "thrust 30000 -> 50 %");

  CHECK(rcFromApp(0, 0, 0, 999).throttle == 0, "thrust below APP_MIN_THRUST is zero");
  CHECK(rcFromApp(0, 0, 0, 65535).throttle == 1, "thrust above full clamps to 1");
  CHECK(rcFromApp(90, -90, 1000, 0).roll == 1 && rcFromApp(90, -90, 1000, 0).pitch == 1 &&
        rcFromApp(90, -90, 1000, 0).yaw == 1, "over-range sticks clamp");

  auto bad = pk; bad[15] ^= 0x01;
  CHECK(!crtp::parseUdp(bad.data(), bad.size(), f), "bad checksum rejected");
  CHECK(!crtp::parseUdp(pk.data(), 1, f), "runt packet rejected");

  auto other = pk; other[0] = 0x70;  // port 7 (generic setpoint / height hold)
  uint8_t sum = 0; for (int i = 0; i < 15; i++) sum += other[i]; other[15] = sum;
  CHECK(crtp::parseUdp(other.data(), other.size(), f) && !crtp::parseCommander(f, c), "non-commander port not treated as sticks");

  auto rsv = pk; rsv[0] = 0x3C;  // reserved bits set (python cflib style)
  sum = 0; for (int i = 0; i < 15; i++) sum += rsv[i]; rsv[15] = sum;
  CHECK(crtp::parseUdp(rsv.data(), rsv.size(), f) && crtp::parseCommander(f, c), "reserved header bits ignored");

  float nan = NAN;
  auto np = appPacket(0, 0, 0, 0); memcpy(&np[1], &nan, 4);
  sum = 0; for (int i = 0; i < 15; i++) sum += np[i]; np[15] = sum;
  CHECK(crtp::parseUdp(np.data(), np.size(), f) && !crtp::parseCommander(f, c), "NaN setpoint rejected");

  uint8_t out[40];
  size_t n = crtp::buildConsole("ARMED via App\n", 14, out);
  CHECK(n == 16 && out[0] == 0x00 && crtp::parseUdp(out, n, f) && f.port == 0 && f.len == 14, "console packet round-trips");
  n = crtp::buildConsole("0123456789012345678901234567890123456789", 40, out);
  CHECK(n == 32, "console chunk capped at 30 chars");
}

// ---------------------------------------------------------------- app arming
// thrustLocked emulation: the only "arm switch" the phone app has.
static void testAppArm() {
  puts("[app arm]");
  AppArm a;
  CHECK(!a.armSwitch(0), "fresh state is disarmed");

  // Connect with the stick already up: must NOT arm until a zero is seen.
  uint32_t t = 1000;
  for (int k = 0; k < 20; k++) { a.feed(30000, t); t += 20; }
  CHECK(!a.armSwitch(t), "thrust>0 right after connect does not arm (locked)");

  a.feed(0, t); t += 20;
  CHECK(!a.armSwitch(t), "zero thrust unlocks but does not arm by itself");
  a.feed(20000, t);
  CHECK(a.armSwitch(t), "raising thrust after a zero arms");
  CHECK(a.armSwitch(t + APP_DISARM_HOLD_MS - 1), "stays armed within the hold window");

  // Thrust back at zero: disarm only after the hold time.
  t += 20; a.feed(0, t);
  CHECK(a.armSwitch(t + APP_DISARM_HOLD_MS - 30), "brief zero thrust keeps it armed");
  CHECK(!a.armSwitch(t + APP_DISARM_HOLD_MS + 1), "zero thrust for the hold time disarms");
  t += APP_DISARM_HOLD_MS + 1;
  a.feed(15000, t);
  CHECK(a.armSwitch(t), "re-arm without a new zero while still unlocked (ESP-Drone behaviour)");

  // Link gap > failsafe: relocks, a stale lastThrustMs cannot re-arm.
  t += FAILSAFE_TIMEOUT_MS + 1;
  a.feed(30000, t);
  CHECK(!a.armSwitch(t), "reconnect with thrust>0 is locked again");
  for (int k = 0; k < 10; k++) { t += 20; a.feed(30000, t); }
  CHECK(!a.armSwitch(t), "stays locked while thrust never returns to zero");
  t += 20; a.feed(APP_MIN_THRUST - 1, t);
  t += 20; a.feed(APP_MIN_THRUST, t);
  CHECK(a.armSwitch(t), "thrust below APP_MIN_THRUST counts as zero; at the threshold counts as raised");

  // millis() wrap-around across the hold window.
  AppArm w;
  uint32_t tw = 0xFFFFFFF0u;
  w.feed(0, tw); w.feed(20000, tw + 8);
  CHECK(w.armSwitch(tw + 8 + 100), "arm state survives millis() wrap");
  CHECK(!w.armSwitch(tw + 8 + APP_DISARM_HOLD_MS + 1), "disarm timing survives millis() wrap");
}

// ---------------------------------------------------------------- closed loop
// Crude single-axis model of a ~40 g brushed quad. NOT a substitute for real
// tuning — it only catches sign errors and wildly wrong gain scales.
struct Axis { float angle = 0, rate = 0, thrustDiff = 0; };

static bool simulate(float startDeg, float targetDeg, float kpRate, float &finalErr,
                     float &peakRate, bool verbose) {
  const float dt = 1.0f / LOOP_HZ;
  const float ACCEL_PER_CMD = 90000.0f;  // deg/s^2 per unit mixer command (model guess)
  const float MOTOR_TAU = 0.03f;         // s
  Axis a;
  a.angle = startDeg;
  Pid rate(kpRate, RATE_RP_KI, RATE_RP_KD, PID_I_LIMIT, PID_OUT_LIMIT, DTERM_LPF_HZ);
  peakRate = 0;
  float gyroFilt = 0;
  float alpha = dt / (1.0f / (2 * PI * GYRO_LPF_HZ) + dt);
  for (int k = 0; k < 3 * LOOP_HZ; k++) {
    gyroFilt += alpha * (a.rate - gyroFilt);
    float sp = constrain(ANGLE_KP * (targetDeg - a.angle), -MAX_RATE_RP_DPS, MAX_RATE_RP_DPS);
    float cmd = rate.update(sp, gyroFilt, dt, true);
    // mixer: differential thrust between the two sides = 2*cmd, clamp by idle/max
    float o[4];
    mixQuadX(0.45f, cmd, 0, 0, o, MOTOR_IDLE);
    float applied = ((o[0] + o[3]) - (o[1] + o[2])) / 4.0f;
    a.thrustDiff += (applied - a.thrustDiff) * dt / MOTOR_TAU;
    a.rate += a.thrustDiff * ACCEL_PER_CMD * dt - a.rate * 0.5f * dt;  // tiny aero damping
    a.angle += a.rate * dt;
    peakRate = std::max(peakRate, std::fabs(a.rate));
    if (verbose && k % 50 == 0) printf("    t=%.1fs angle=%6.2f rate=%7.1f cmd=%.3f\n", k * dt, a.angle, a.rate, cmd);
    if (!std::isfinite(a.angle) || std::fabs(a.angle) > 180) { finalErr = 999; return false; }
  }
  finalErr = std::fabs(targetDeg - a.angle);
  return finalErr < 2.0f;
}

static void testClosedLoop(bool verbose) {
  puts("[closed-loop sim]");
  float err, peak;
  bool ok = simulate(20, 0, RATE_RP_KP, err, peak, verbose);
  printf("    default gains: 20deg -> 0deg, final error %.2f deg, peak rate %.0f deg/s\n", err, peak);
  CHECK(ok, "default gains level the drone from 20 deg within 3 s (model)");
  ok = simulate(0, 25, RATE_RP_KP, err, peak, false);
  CHECK(ok, "default gains track a 25 deg stick command (model)");
  CHECK(peak <= MAX_RATE_RP_DPS * 1.3f, "rate stays near the configured limit");

  // Wrong sign must diverge — proves the test would catch a sign bug.
  ok = simulate(20, 0, -RATE_RP_KP, err, peak, false);
  CHECK(!ok, "inverted P gain is detected as unstable");
}

int main(int argc, char **argv) {
  bool verbose = argc > 1 && std::strcmp(argv[1], "-v") == 0;
  testMixer();
  testPid();
  testAhrs();
  testSticks();
  testApp();
  testAppArm();
  testClosedLoop(verbose);
  printf("\n%d checks, %d failed -> %s\n", checks, failures, failures ? "FAILED" : "ALL PASSED");
  return failures ? 1 : 0;
}
