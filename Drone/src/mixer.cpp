#include "mixer.h"
#include "config.h"

void mixQuadX(float t, float r, float p, float y, float out[4], float idle) {
  // roll+  (right down): speed up left motors
  // pitch+ (nose up):    speed up front motors
  // yaw+   (nose right): speed up CCW motors (their reaction torque turns frame CW)
  float c[4];
  c[0] = +r + p - y;  // M1 front-left  CW
  c[1] = -r + p + y;  // M2 front-right CCW
  c[2] = -r - p - y;  // M3 rear-right  CW
  c[3] = +r - p + y;  // M4 rear-left   CCW

  // If the corrections don't fit above the throttle, first shift the whole set
  // down (keeps the full differential) - but only by MIX_MAX_SHIFT. Beyond that
  // scale the corrections instead: losing some attitude authority beats losing
  // most of the thrust, which is what an unlimited shift does when all three
  // PIDs saturate (seen on the bench with the drone held: 1.00/0.06/0.06/0.06).
  float hi = c[0];
  for (int i = 1; i < 4; i++) if (c[i] > hi) hi = c[i];
  float room = MOTOR_MAX - t;                 // headroom above throttle
  float shift = 0;
  if (hi > room) {
    shift = hi - room;
    if (shift > MIX_MAX_SHIFT) {
      float scale = (room + MIX_MAX_SHIFT) / hi;  // hi > 0 here
      for (int i = 0; i < 4; i++) c[i] *= scale;
      shift = MIX_MAX_SHIFT;
    }
  }
  for (int i = 0; i < 4; i++) {
    out[i] = t + c[i] - shift;
    if (out[i] < idle) out[i] = idle;
    if (out[i] > MOTOR_MAX) out[i] = MOTOR_MAX;
  }
}
