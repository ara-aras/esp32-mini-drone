#pragma once
#include <Arduino.h>

struct Telemetry {
  float roll, pitch;          // deg
  float gx, gy, gz;           // deg/s (filtered)
  float accNorm;              // |accel| in g (~1.0 at rest; AHRS ignores accel outside 0.8..1.2)
  float sp[3];                // rate setpoints roll/pitch/yaw (deg/s)
  float pid[3];               // PID outputs roll/pitch/yaw
  float motor[4];             // 0..1
  float throttle;             // stick 0..1
  bool  armed;
  bool  benchBusy;
  uint32_t rxAge, rxPackets, imuErrors;
  float loopHz;
  uint32_t workUs;            // peak control-loop compute time
  uint32_t stackFree;         // flight task stack high-water mark (bytes)
  uint32_t stampMs;           // millis() when published; loop() watchdog checks it
  uint8_t  rcSource;          // RcSource
};

void      telemetryPublish(const Telemetry &t);
Telemetry telemetryGet();
