#pragma once
#include <Arduino.h>
#include "imu.h"
#include "settings.h"

// Bench tests. Requested from the loop task (serial/web), executed inside the
// flight task ONLY while disarmed, so I2C and motors stay single-owner.
enum class BenchJob : uint8_t {
  None,
  Calibrate,   // gyro bias calibration
  SelfTest,    // arg 0 = sensors/logic only, arg 1 = + motor spin check
  Orient,      // guided tilt test: verifies IMU axis signs
  MotorOne,    // arg = motor index 0..3
  MotorSeq,    // M1..M4 one after another
  MotorRamp,   // arg = index 0..3, or -1 for all
  MixTest,     // arg 0 roll, 1 pitch, 2 yaw: shows mixer direction
  I2cScan,
};

// ---- loop-task side ----
bool benchRequest(BenchJob job, int arg, bool armed, String &err);
void benchAbort();
bool benchBusy();
void benchUnlockMotors();          // "props off"
void benchLockMotors();
uint32_t benchUnlockRemainingMs();

// ---- flight-task side ----
struct BenchCtx {
  const ImuSample &s;
  float roll, pitch, dt, loopHz;
  uint32_t imuErrors, workUs, stackFree;
  const Settings &st;
  bool timingReset;   // set true if the job blocked (caller resyncs its timers)
};
// Fills out[4]; returns true if motors should be driven.
bool benchTick(BenchCtx &c, float out[4]);
// Call when arming: drops a job that was requested in the same instant the
// drone armed (loop() decides "disarmed" from a slightly stale snapshot), so it
// cannot start spinning motors the moment the drone disarms again.
void benchDiscardPending();
