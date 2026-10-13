#pragma once
#include <Arduino.h>

// ESP-Drone phone app link: CRTP commander packets over UDP port 2390.
bool     appLinkBegin();                 // after netBegin()
void     appLinkLoop();                  // from loop(): flushes console text to the app
void     appLinkConsole(const char *s);  // queue text for the app's console view
uint32_t appLinkBadPackets();
