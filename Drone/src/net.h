#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// Starts the Wi-Fi access point (AP+STA so ESP-NOW keeps working).
// IP is fixed to what the ESP-Drone app expects. Call before radioBegin().
bool      netBegin();
String    netSsid();
IPAddress netIp();
int       netClients();
