#pragma once
#include <Arduino.h>

// Body frame: X forward, Y right, Z down. Gyro in deg/s, accel in g.
struct ImuSample {
  float gx, gy, gz;
  float ax, ay, az;
};

bool imuBegin();                 // init MPU6050, returns false if not found
bool imuCalibrateGyro();         // keep drone still! returns false if it moved
bool imuIsCalibrated();
bool imuRead(ImuSample &s);      // bias-corrected, low-pass filtered gyro
