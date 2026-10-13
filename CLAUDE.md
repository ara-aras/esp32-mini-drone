# Context for Claude Code

Custom ESP32 quadcopter flight controller, **Arduino framework, built in Arduino IDE**
(not ESP-IDF, not ESP-Drone). Keep it that way.

## Verify every change
Run `tools/check_all.sh` (unit tests + compile both sketches). Keep it warning-free on
ESP32 core 2.x AND 3.x. Add unit tests in `tests/test_main.cpp` for any logic change in
mixer/pid/ahrs/rcmath/crtp/apparm (those files must stay free of ESP32-only headers).

## Rules
- Hardware constants and defaults live in `Drone/src/config.h`. Runtime-tunable values
  go through `settings.*` (add to the KEYS table; the schema auto-invalidates old flash).
- Sources live in `Drone/src/`; include them from the .ino as `"src/xxx.h"`.
- Version-specific code: LEDC uses `ESP_ARDUINO_VERSION_MAJOR`, ESP-NOW recv callback uses
  `ESP_IDF_VERSION_MAJOR`. Don't register an ESP-NOW *send* callback (signature changed in IDF 5.5).
- `RcPacket` is duplicated in `Drone/src/radio.h` and `Transmitter/Transmitter.ino`.
- Log with `dlog()` (goes to Serial and the web log), not `Serial.print`, except telemetry output.
- Startup order: `netBegin()` (AP, fixed IP 192.168.43.42) → `webBegin()` → `appLinkBegin()`
  → `radioBegin()`. AP channel == `RADIO_WIFI_CHANNEL`.
- ESP-Drone app protocol (from EspressifApps/ESP-Drone-Android): UDP to 192.168.43.42:2390,
  datagram = CRTP header + payload + 8-bit sum. Commander = port 3 ch 0, LE float roll deg,
  float pitch deg (app pre-negates: + = nose up), float yaw deg/s (+ = nose right),
  uint16 thrust. The app needs no replies. Parsing lives in `crtp.h` (host-tested).
- App arming emulates ESP-Drone's thrustLocked: zero thrust after (re)connect, then thrust>0
  arms; zero for APP_DISARM_HOLD_MS disarms. Logic in `apparm.h` (host-tested), used by `radio.cpp`.
- No external libraries.

## Architecture
- Flight loop: FreeRTOS task, core 1, LOOP_HZ (500). It alone touches I2C and motors
  after setup(). Bench tests run inside it via `benchTick()` only while disarmed.
- `loop()` (core 1, low prio): serial commands, web server, LED, battery, telemetry print.
- Commands from serial/web -> `cmdExecute()` -> `benchRequest()` (spinlock) -> flight task.
- Shared data: `telemetryPublish/Get`, `settingsGet/Version`, all spinlock-protected.
- `blackbox.cpp`: flight task writes frames (arm -> record, disarm -> 1 s post-roll ->
  frozen); loop task reads the ring ONLY while frozen (no lock needed) and saves fault
  logs to LittleFS. `loop()` also runs the FLIGHT_WDT_MS motor watchdog.
- `dlog()` never touches Serial (ring only); `loop()` drains it via `logFlushSerial()`.

## Conventions (don't break these silently)
- Body frame X forward, Y right, Z down. roll+ right side down, pitch+ nose up, yaw+ nose right.
- Motors: M1 FL CW, M2 FR CCW, M3 RR CW, M4 RL CCW.
- Sticks: roll+ right, pitch+ forward (maps to nose-down target), yaw+ right.

## Safety
Never weaken arming checks, the failsafe, or the `props off` motor-test lock without the
user explicitly asking. Motor tests only while disarmed.
