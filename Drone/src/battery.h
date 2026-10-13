#pragma once

void  batteryBegin();
void  batteryUpdate();   // call ~10 Hz from a low-priority context
float batteryVoltage();  // filtered volts (0 if sensing disabled)
bool  batteryOkToArm();
bool  batteryLow();
