#include "pid.h"
#include <Arduino.h>

Pid::Pid(float kp, float ki, float kd, float iLimit, float outLimit, float dLpfHz)
    : kp_(kp), ki_(ki), kd_(kd), iLimit_(iLimit), outLimit_(outLimit),
      dRc_(1.0f / (2.0f * 3.14159265f * dLpfHz)) {}

void Pid::setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

void Pid::reset() {
  integral_ = 0;
  dFilt_ = 0;
  first_ = true;
}

float Pid::update(float setpoint, float measurement, float dt, bool integrate) {
  float err = setpoint - measurement;

  if (integrate) {
    integral_ = constrain(integral_ + ki_ * err * dt, -iLimit_, iLimit_);
  } else {
    integral_ = 0;
  }

  float d = 0;
  if (first_) {
    first_ = false;
  } else if (dt > 0) {
    float raw = -(measurement - prevMeas_) / dt;
    dFilt_ += (dt / (dRc_ + dt)) * (raw - dFilt_);
    d = dFilt_;
  }
  prevMeas_ = measurement;

  return constrain(kp_ * err + integral_ + kd_ * d, -outLimit_, outLimit_);
}
