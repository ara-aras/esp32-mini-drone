#include "ahrs.h"
#include "config.h"
#include <math.h>

namespace {
const float RAD2DEG = 57.2957795f;  // float: the ESP32 FPU has no double
float roll = 0, pitch = 0;
float trimR = ACC_TRIM_ROLL_DEG, trimP = ACC_TRIM_PITCH_DEG;

void accelAngles(const ImuSample &s, float &r, float &p) {
  // Specific force at rest, level, Z-down frame: (0, 0, -1g)
  r = atan2f(-s.ay, -s.az) * RAD2DEG - trimR;
  p = atan2f(s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD2DEG - trimP;
}
}  // namespace

void ahrsReset(const ImuSample &s) { accelAngles(s, roll, pitch); }

void ahrsUpdate(const ImuSample &s, float dt) {
  roll  += s.gx * dt;
  pitch += s.gy * dt;

  float norm = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  if (norm > 0.8f && norm < 1.2f) {  // ignore accel during hard manoeuvres
    float ar, ap;
    accelAngles(s, ar, ap);
    roll  = COMPL_ALPHA * roll  + (1.0f - COMPL_ALPHA) * ar;
    pitch = COMPL_ALPHA * pitch + (1.0f - COMPL_ALPHA) * ap;
  }
}

float ahrsRoll()  { return roll; }
float ahrsPitch() { return pitch; }
void  ahrsSetTrim(float r, float p) { trimR = r; trimP = p; }
