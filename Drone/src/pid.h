#pragma once

class Pid {
 public:
  Pid(float kp, float ki, float kd, float iLimit, float outLimit, float dLpfHz);
  // D term acts on measurement (no kick on setpoint changes).
  float update(float setpoint, float measurement, float dt, bool integrate);
  void  reset();
  void  setGains(float kp, float ki, float kd);

 private:
  float kp_, ki_, kd_, iLimit_, outLimit_, dRc_;  // dRc_ = 1/(2*pi*fc), precomputed
  float integral_ = 0, prevMeas_ = 0, dFilt_ = 0;
  bool  first_ = true;
};
