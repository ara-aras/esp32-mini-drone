#pragma once

// Quad-X mixer. Inputs: throttle 0..1, roll/pitch/yaw corrections (motor units).
// Output: 4 motor levels in [MOTOR_IDLE, MOTOR_MAX], order M1..M4 (see config.h).
// Output: 4 motor levels in [idle, MOTOR_MAX].
void mixQuadX(float throttle, float roll, float pitch, float yaw, float out[4], float idle);
