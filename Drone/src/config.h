// config.h — ALL hardware-specific and tuning settings live here.
// If your hardware changes, you should only need to edit this file.
#pragma once
#include <Arduino.h>

// ============================================================
// Pins (classic ESP32 DevKit / ESP32-WROOM-32)
// Avoid strapping/boot-glitch pins (0, 2*, 5, 12, 14, 15) for motors.
// Battery sense MUST be an ADC1 pin (32-39); ADC2 is unusable with Wi-Fi on.
// ============================================================
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define I2C_FREQ_HZ   400000

// Motor layout (top view, nose up):
//
//      M1 (CW)     M2 (CCW)
//          \       /
//           [ FC ]
//          /       \   .
//      M4 (CCW)    M3 (CW)
//
// M1 front-left, M2 front-right, M3 rear-right, M4 rear-left.
// Matched to the as-built wiring (mall test 2026-09-17): FL=32/18 FR=25/16 RR=33/19 RL=26/17
#define PIN_MOTOR1    32
#define PIN_MOTOR2    25
#define PIN_MOTOR3    33
#define PIN_MOTOR4    26

#define PIN_LED       2
#define PIN_VBAT      34

// ============================================================
// Motor output
// ============================================================
#define MOTOR_TYPE_BRUSHED  0   // coreless brushed motors (DRV8833 or MOSFETs)
#define MOTOR_TYPE_ESC_PWM  1   // brushless ESCs, standard 1000-2000us PWM
#define MOTOR_TYPE          MOTOR_TYPE_BRUSHED

// DRV8833 wiring (two boards, one per side). Each motor uses IN1 + IN2:
//   PIN_MOTORx -> IN1 (PWM),  PIN_MOTORx_IN2 -> IN2 (held LOW = forward, coast when off)
// Set an IN2 pin to -1 if you tied that IN2 to GND on the board instead.
// Spinning the wrong way? Swap that motor's two wires (OUT1/OUT2).
//   As built: board with 25/16 + 33/19 drives M2 FR + M3 RR (right side);
//             board with 26/17 + 32/18 drives M4 RL + M1 FL (left side).
// VM = battery +, GND common with the ESP32.
#define USE_DRV8833         1
#define PIN_MOTOR1_IN2      18
#define PIN_MOTOR2_IN2      16
#define PIN_MOTOR3_IN2      19
#define PIN_MOTOR4_IN2      17
// Motor spins the wrong way? Instead of swapping its OUT wires, set its flag to 1:
// the PWM then goes to IN2 and IN1 is held low. Order: M1 FL, M2 FR, M3 RR, M4 RL.
// Needs both IN pins wired (not -1). Verify with `mall`: M1,M3 CW; M2,M4 CCW.
#define MOTOR_REVERSE       {1, 0, 1, 0}   // as built: FL and RR were CCW
// SLP/nSLEEP/EEP pin of BOTH boards (tie together). Held LOW until PWM is ready,
// so motors can't twitch at boot. Set -1 if your board has no such pin.
#define PIN_DRV_SLEEP       23

#define BRUSHED_PWM_FREQ    16000   // Hz (12-bit is the max resolution here)
#define BRUSHED_PWM_BITS    12
#define ESC_PWM_FREQ        400     // Hz (use 50 if your ESC is fussy)
#define ESC_PWM_BITS        14
#define ESC_MIN_US          1000
#define ESC_MAX_US          2000

#define MOTOR_IDLE          0.06f   // output while armed at zero throttle (0..1)
#define MOTOR_MAX           1.00f
#define MOTOR_TEST_LEVEL    0.15f   // default bench test level (tunable: test_level)
#define MOTOR_TEST_MS       1500
#define MIX_TEST_DELTA      0.06f   // extra command applied in the mixer direction test
#define MIX_MAX_SHIFT       0.25f   // max thrust the mixer may give up to keep a correction; beyond, corrections are scaled
#define PROPS_OFF_UNLOCK_MS 300000  // "props off" unlocks motor tests for 5 minutes

// ============================================================
// IMU (MPU6050)
// Body frame used everywhere: X forward, Y right, Z down.
// roll+ = right side down, pitch+ = nose up, yaw+ = nose right.
// ============================================================
#define IMU_ADDR            0x68    // 0x69 if AD0 is pulled high
#define I2C_TIMEOUT_MS      5       // a stalled bus must not hold the motors (one read < 1 ms)
#define IMU_FAIL_DISARM_STREAK 3    // consecutive failed reads while armed -> disarm

// Chip axes -> body axes. Default: board flat, components up,
// the printed X arrow pointing to the drone's nose.
// If mounted differently, change ONLY this macro.
#define IMU_MAP(cx, cy, cz, bx, by, bz) \
  do { bx = (cx); by = -(cy); bz = -(cz); } while (0)

#define GYRO_CAL_SAMPLES      1000
#define GYRO_CAL_MAX_STD_DPS  1.5f  // calibration rejected if drone is moving
#define GYRO_LPF_HZ           80.0f
#define ACC_TRIM_ROLL_DEG     0.0f  // default trims (tunable: trim_roll / trim_pitch)
#define ACC_TRIM_PITCH_DEG    0.0f
#define COMPL_ALPHA           0.995f // gyro trust in complementary filter

// ============================================================
// Control loop
// ============================================================
#define LOOP_HZ               500
#define FLIGHT_TASK_CORE      1     // Wi-Fi/ESP-NOW runs on core 0
#define FLIGHT_TASK_PRIO      5
#define FLIGHT_WDT_MS         50    // loop() cuts motors if the flight task stops publishing this long

// Angle mode (self-levelling) outer loop
#define MAX_ANGLE_DEG         30.0f
#define ANGLE_KP              4.0f   // deg error -> deg/s setpoint
#define MAX_RATE_RP_DPS       200.0f
#define MAX_RATE_YAW_DPS      180.0f

// Inner rate loop. Output units = motor fraction (0..1).
// These are DEFAULTS. Live values are tuned with `set` and stored in flash
// with `save` (see settings.cpp). STARTING GUESSES ONLY.
#define RATE_RP_KP            0.0012f
#define RATE_RP_KI            0.0010f
#define RATE_RP_KD            0.00002f
#define RATE_YAW_KP           0.0030f
#define RATE_YAW_KI           0.0010f
#define RATE_YAW_KD           0.0f
#define PID_I_LIMIT           0.10f
#define PID_OUT_LIMIT         0.30f
#define DTERM_LPF_HZ          40.0f
#define I_TERM_MIN_THROTTLE   0.15f  // integrators frozen below this stick level

// ============================================================
// Radio (ESP-NOW) — must match Transmitter.ino
// ============================================================
#define RADIO_WIFI_CHANNEL    1
#define RADIO_MAGIC           0xD5A1
#define FAILSAFE_TIMEOUT_MS   500    // no packet this long while armed -> disarm
#define STICK_DEADBAND        20     // on a +/-500 scale

// ============================================================
// Safety / battery (1S LiPo assumed)
// ============================================================
#define VBAT_SENSE_ENABLED    0      // TEMPORARY: no divider yet. Set back to 1 before flying!
#define VBAT_DIVIDER          2.0f   // 100k/100k divider
#define VBAT_CALIBRATION      1.00f  // tweak after comparing with a multimeter
#define VBAT_MIN_ARM          3.50f
#define VBAT_LOW_WARN         3.30f

#define ARM_MAX_THROTTLE      0.05f
#define ARM_THROTTLE_RAMP_MS  300    // throttle cap rises 0 -> 100 % over this after arming (soft start)
#define ARM_MAX_TILT_DEG      25.0f
#define CRASH_TILT_DEG        70.0f

// ============================================================
// Flight recorder (blackbox) — see blackbox.h
// ============================================================
#define BLACKBOX_HZ           100    // frames per second while armed (LOOP_HZ must divide)
#define BLACKBOX_FRAMES       1400   // 36 B each -> ~50 KB RAM, 14 s of history
#define BLACKBOX_POST_MS      1000   // keep recording this long after a disarm
#define BLACKBOX_PERSIST      1      // save fault logs to LittleFS (spiffs partition)

// ============================================================
// Self-test thresholds
// ============================================================
#define ST_GYRO_NOISE_OK_DPS   1.5f
#define ST_GYRO_NOISE_BAD_DPS  4.0f
#define ST_GYRO_BIAS_OK_DPS    2.0f
#define ST_LEVEL_OK_DEG        5.0f
#define ST_MIN_FREE_HEAP       40000
#define ST_MIN_FLIGHT_STACK    1024   // bytes of flight-task stack never touched
#define ST_VIB_FACTOR          2.0f   // spin vibration must exceed baseline*factor+offset
#define ST_VIB_OFFSET_G        0.01f
#define ST_SAG_MIN_V           0.03f  // battery sag that proves a motor draws current
#define ORIENT_STEP_TIMEOUT_MS 15000

// ============================================================
// Control links
// ============================================================
#define RADIO_ESPNOW_ENABLED 1   // ESP32 joystick transmitter (Transmitter/ sketch)
#define APP_LINK_ENABLED     1   // ESP-Drone phone app (Android/iOS) over Wi-Fi
// If both are active, a fresh ESP-NOW transmitter wins; the app is used otherwise.

// ============================================================
// Wi-Fi access point (used by the ESP-Drone app AND the web dashboard)
// Phone: join ESP-DRONE_<MAC>, open the ESP-Drone app, press Connect.
// Dashboard: http://192.168.43.42
// ============================================================
#define WIFI_AP_SSID_BASE    "ESP-DRONE"   // final SSID: ESP-DRONE_<12 hex MAC>
// NOT ESP-Drone's public default (12345678): anyone knowing it could open the dashboard
// and spin motors. The app does not care which password you use. Min 8 chars.
// Put your own in src/secrets.h (git-ignored):  #define WIFI_AP_PASS "your-pass"
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef WIFI_AP_PASS
#define WIFI_AP_PASS         "change-me-now"
#endif
#define WIFI_AP_MAX_CLIENTS  3
#define WEB_ENABLED          1

// ---- ESP-Drone app protocol (CRTP over UDP). The app HARD-CODES these: ----
#define APP_DEVICE_IP        192, 168, 43, 42
#define APP_UDP_PORT         2390
#define APP_MIN_THRUST       1000     // below this = thrust zero (same as ESP-Drone)
#define APP_THRUST_FULL      60000    // thrust value mapped to 100 % throttle
// Full stick in the app = these values. Match them to the app's settings
// ("max roll/pitch angle" default 15, "max yaw angle" default 90). The drone's
// own max_angle / max_rate_yaw settings then decide the real limits.
#define APP_FULL_ANGLE_DEG   15.0f
#define APP_FULL_YAW_DPS     90.0f
// The app has no arm button. Like ESP-Drone: thrust must be seen at zero after
// connecting; raising thrust arms; thrust back at zero for this long disarms.
#define APP_DISARM_HOLD_MS   300
