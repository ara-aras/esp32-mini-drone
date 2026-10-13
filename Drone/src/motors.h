#pragma once

bool motorsBegin();                 // configures PWM, motors start stopped
void motorsWrite(const float lvl[4]); // 0 = stopped, 1 = full
void motorsWriteOne(int idx, float lvl);
void motorsStop();
