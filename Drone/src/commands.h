#pragma once
#include <Arduino.h>

enum class TelemMode : uint8_t { Off, Text, PlotRoll, PlotPitch, PlotYaw, Attitude };

// One command line from Serial or the web dashboard. Output goes to dlog().
void      cmdExecute(const String &line, bool armed);
TelemMode cmdTelemMode();
