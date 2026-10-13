#pragma once
#include "imu.h"

// Complementary filter giving roll/pitch in degrees (body frame convention
// in config.h). Yaw is not estimated: yaw is flown in rate mode.
void  ahrsReset(const ImuSample &s);
void  ahrsUpdate(const ImuSample &s, float dt);
float ahrsRoll();
float ahrsPitch();
void  ahrsSetTrim(float rollDeg, float pitchDeg);
