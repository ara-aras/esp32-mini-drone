# ESP32 Drone — custom Arduino IDE firmware

Self-written quadcopter flight controller for a classic ESP32, flashed from Arduino IDE.
Angle (self-levelling) mode on roll/pitch, rate mode on yaw. Fly it with the official
**ESP-Drone phone app** (Wi-Fi) or an ESP-NOW transmitter made from a second ESP32, plus a full **bench test suite**, **live PID tuning saved to flash**, and a
**Wi-Fi web dashboard** hosted by the drone.

**Verified:** compiles with `-Wall` and no warnings on ESP32 Arduino core **2.0.9** and
**3.3.11** (board "ESP32 Dev Module"). 67 host unit tests pass.
**Not flight-tested.** PID gains are starting guesses.

## Folder layout
```
Drone/                 open Drone.ino in Arduino IDE, flash to the drone
  Drone.ino            flight task, arming, failsafe, serial, LED
  src/config.h         EVERY pin, motor type, IMU orientation, default gain, threshold
  src/imu.*            MPU6050 driver (raw I2C)
  src/ahrs.*           complementary filter -> roll/pitch
  src/pid.*            PID, anti-windup, D-on-measurement
  src/mixer.*          quad-X mixer
  src/motors.*         LEDC PWM, brushed or ESC, core 2.x + 3.x
  src/radio.*          control input: ESP-NOW + app, source selection, app arming
  src/applink.*        ESP-Drone app link (UDP 2390)   src/crtp.h  its packet parser
  src/net.*            Wi-Fi access point (fixed IP the app expects)
  src/rcmath.h         stick maths
  src/battery.*        1S battery sensing
  src/settings.*       live-tunable values, stored in flash (NVS)
  src/bench.*          self-test, orientation test, motor/mixer tests, I2C scan
  src/commands.*       command interpreter (shared by Serial and web)
  src/web.* web_page.h Wi-Fi AP + dashboard
  src/telemetry.* logbuf.*   shared state and log buffer
Transmitter/           flash to the second ESP32 (joysticks + arm switch)
tests/                 PC unit tests + closed-loop PID simulation (`make`)
tools/check_all.sh     unit tests + compile both sketches with arduino-cli
CLAUDE.md              context for Claude Code
PPT/Drone_Workshop_v2.pptx  IEEE workshop slides (speaker notes included)
```
No external libraries needed.

## Hardware assumed (change in config.h)
| Item | Default |
|---|---|
| Board | ESP32 DevKit (WROOM-32) |
| IMU | MPU6050, SDA 21, SCL 22, flat, X arrow to the nose |
| Motors | coreless brushed via **2× DRV8833** (IN1 = PWM, IN2 held LOW) |
| Motor IN1 (PWM) | M1 FL 25, M2 FR 26, M3 RR 32, M4 RL 33 |
| Motor IN2 (LOW) | M1 16, M2 17, M3 18, M4 19 (or tie IN2 to GND and set -1) |
| DRV8833 SLP/EEP | GPIO23, both boards (set -1 if absent) |
| Props | M1 & M3 clockwise, M2 & M4 counter-clockwise (top view) |
| Battery sense | GPIO34 via 100k/100k divider from 1S LiPo |
| LED | GPIO2 |

Brushless? Set `MOTOR_TYPE` to `MOTOR_TYPE_ESC_PWM`. Do NOT use L298N-type drivers.

**DRV8833 notes:** each channel is rated about 1.5 A continuous. Small 7×16 / 8×20 coreless
motors can exceed that at full throttle or stall, and the driver will thermal-limit
(motors cut out mid-air). Check your motor's current, keep test levels low, and feel the
chips after `ramp all`. A motor spinning backwards → swap its OUT1/OUT2 wires.

## Arduino IDE settings
Board **ESP32 Dev Module**, Serial Monitor **115200**, line ending **Newline**.
Default partition scheme is fine (firmware uses ~77 % of it on core 3.x).

## Flying with the ESP-Drone app
Implemented from the app's source code (EspressifApps/ESP-Drone-Android): the app sends
CRTP commander packets over UDP to `192.168.43.42:2390` every 20 ms.

1. Install **ESP-Drone** (Android / iOS App Store).
2. Power the drone. Join Wi-Fi **`ESP-DRONE_<MAC>`**, password = `WIFI_AP_PASS`. Set your own in `Drone/src/secrets.h` (git-ignored):
   `#define WIFI_AP_PASS "your-pass"` (min 8 chars; the app accepts any password). Without it the
   default `change-me-now` is used - change it, anyone who knows it can spin your motors
   (the SSID is printed on Serial at boot).
3. **Turn mobile data off** (or tell Android to stay on this network without internet),
   otherwise the phone may route the UDP packets away from the drone.
4. Open the app → **Connect**. `status` on Serial should show `control App`.
5. **Arming:** there is no arm button. Throttle must first be at zero after connecting;
   pushing throttle up arms (if all checks pass); throttle back to zero for 0.3 s disarms.
   **Turn mobile data off** on the phone (or the app's UDP goes out over cellular and it
   reports "no drone found" even though Wi-Fi is connected).
   Closing the app or losing Wi-Fi disarms within 0.5 s.
6. In app settings: keep **height-hold OFF** (not supported — those packets are ignored
   and the drone disarms). Leave "max roll/pitch angle" at 15 and "max yaw" at 90, or
   change `APP_FULL_ANGLE_DEG` / `APP_FULL_YAW_DPS` to match.
7. Enable "show console" in the app to see ARMED / ARM REFUSED / DISARMED messages.

Note: the app jumps straight to its "min thrust" (default 25 %) once the stick leaves
the deadzone. Lower it in the app if takeoff feels abrupt.

If an ESP-NOW transmitter is also on and sending, it takes priority over the app.

## Testing tools
- **Serial Monitor:** type `help`.
- **Dashboard:** on the same Wi-Fi, open **http://192.168.43.42**. Live attitude, motor
  bars, control source, all test buttons, a PID graph, the tuning table and the log.
  While armed, only `status/tel/plot/get/stop` work.

## Command reference
| Command | What it does |
|---|---|
| `status` | versions, state, loop rate, radio, battery, web address |
| `selftest` | ~15 checks: calibration, loop rate & timing, IMU errors, battery, radio link & stick positions, heap, mixer logic, PID logic, failsafe, gyro noise/bias, accel scale, level |
| `selftest motors` | the above + spins each motor and confirms it responds via vibration / battery sag |
| `orient` | guided: tilt right, nose up, yaw right → confirms IMU axis signs, tells you what to fix in `IMU_MAP` |
| `cal` | recalibrate gyro |
| `i2c` | scan bus (IMU should show at 0x68) |
| `props off` | you confirm props are removed → motor tests unlocked 5 min (`lock` to relock) |
| `t1`…`t4` | spin one motor |
| `mall` | spin M1→M4 in order: check position and direction |
| `ramp [1-4\|all]` | smooth up/down ramp: spot a bad motor or a PWM problem |
| `mix roll\|pitch\|yaw` | applies a +/− correction through the real mixer, and prints which motors should speed up |
| `tel` | 10 Hz text telemetry |
| `plot roll\|pitch\|yaw\|off` | Arduino **Serial Plotter**: setpoint vs gyro vs PID output (for tuning) |
| `get` / `set key value` | view/change gains, limits, idle, trims, test level (live) |
| `save` / `load` / `defaults` | flash storage |
| `stop` | abort any test immediately (also the STOP button) |
| `bb` / `bb saved` / `bb clear` | dump the flight recorder as CSV (last flight / the fault log kept in flash) / delete it |

Motor tests only run while disarmed, and arming is refused while a test runs.

## Bring-up checklist — PROPS OFF until step 8
1. `tools/check_all.sh` (or just `make` in `tests/`) → all unit tests pass.
2. Flash `Drone`. Serial shows WHO_AM_I, gyro bias, `[web] AP ... up`, `Ready`.
3. `i2c` → device at 0x68.
4. `selftest` → no FAIL (a radio WARN is expected before the transmitter is on).
5. `orient` → three PASS. If not, fix `IMU_MAP` and reflash.
6. `props off`, then `mall` → each motor in the right corner, right direction.
   `selftest motors` → four PASS. `mix roll` / `mix pitch` / `mix yaw` → the motors the log names are the ones that speed up.
7. Connect the app. `selftest` again → control link PASS via App.
   Move the right stick: `tel` shows roll/pitch change the right way. Raise throttle →
   `ARMED via App`. Tilt the frame by hand: the low side's motors must speed up.
   Close the app → `DISARMED: FAILSAFE` within 0.5 s. Throttle to zero → disarms.
8. Props on, tethered rig. `plot roll`, tune `rp_kp` → `rp_kd` → `rp_ki` with `set`,
   `save` when happy.

## Finding faults: log + flight recorder
Every log line carries a `[sec.ms]` timestamp. Arm/disarm (with the reason), arm
refusals, link lost/OK, control-source changes, battery low/ok, bench results and
the watchdog all log. The web log and Serial show the same ring (last ~6 KB).

Text can't describe *why* a flight went wrong - numbers can. While armed, the
**flight recorder** stores a frame every 10 ms (attitude, gyro, rate setpoints, PID
outputs, motor outputs, throttle, sticks, battery, link/source flags): ~14 s of
history, plus 1 s after the disarm. Then it freezes until the next arm.

- Dashboard → **Flight recorder → Download last flight** (or `bb` on Serial) →
  `blackbox.csv`. Open in a spreadsheet, plot `sp_roll` vs `gx` vs `pid_roll` and the
  four motor columns around the disarm.
- A **fault** disarm (failsafe, crash tilt, IMU failure, source change, watchdog)
  is also written to flash together with the event log, because a crash usually
  pops the battery. It survives reboot: **Download saved fault log** / `bb saved`
  (`bb clear` deletes it; `status` says if one exists). The first boot formats
  LittleFS - a few seconds, once.
- The last lines of the events section (`# ...` in the saved CSV) tell you the reason;
  the frames tell you what the controller saw and did in the seconds before.

## Safety behaviour
- Won't arm unless: link OK, arm switch cycled OFF→ON (app: thrust seen at zero, then
  raised), throttle low (transmitter), tilt < 25°, gyro calibrated, battery ≥ 3.5 V,
  no bench test running.
- Disarms on: switch off / app throttle at zero, 500 ms without packets, tilt > 70°,
  3 consecutive IMU read failures, control source changing mid-flight, or the loop
  task going silent for 50 ms (the `loop()` watchdog cuts the motors itself).
- After **any** disarm the switch must be cycled again (app: thrust to zero); a link
  dropout never counts as a cycle, so a failsafe can't be followed by a silent re-arm.
- Throttle is ramped in over 0.3 s after arming (soft start).
- Failsafe cuts motors (it falls). Fine for learning; stay low.

## Limits of the tests (be honest with yourself)
- `selftest motors` proves a motor draws current / vibrates, not its direction. Look.
- The closed-loop simulation uses a guessed model. It catches sign errors and absurd
  gains, not real-world tuning.
- The app link is not tested against a real phone yet; the packet parser is tested
  against packets built exactly as the app's source builds them.
- Wi-Fi latency on phones varies. Set `WEB_ENABLED 0` and close the dashboard while
  flying if control feels laggy.
