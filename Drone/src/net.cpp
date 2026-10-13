#include "net.h"
#include "config.h"
#include "logbuf.h"
#include <WiFi.h>
#include <esp_mac.h>

namespace {
String ssid;
bool up = false;
}

bool netBegin() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);  // lower latency
  // Read the AP MAC from efuse: WiFi.softAPmacAddress() is all zeros until the
  // AP is actually up on core 3.x, and the SSID is needed before that.
  uint8_t m[6];
  esp_read_mac(m, ESP_MAC_WIFI_SOFTAP);
  char mac[13];
  snprintf(mac, sizeof(mac), "%02X%02X%02X%02X%02X%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
  ssid = String(WIFI_AP_SSID_BASE) + "_" + mac;

  if (!WiFi.softAP(ssid.c_str(), WIFI_AP_PASS, RADIO_WIFI_CHANNEL, 0, WIFI_AP_MAX_CLIENTS)) {
    dlog("[net] softAP failed (password must be >= 8 chars)\n");
    return false;
  }
  delay(100);
  IPAddress ip(APP_DEVICE_IP);
  if (!WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0))) {
    dlog("[net] softAPConfig failed - the app will not find the drone\n");
  }
  up = true;
  dlog("[net] Wi-Fi '%s' (pass %s) on ch %d, IP %s\n", ssid.c_str(), WIFI_AP_PASS,
       RADIO_WIFI_CHANNEL, WiFi.softAPIP().toString().c_str());
  return true;
}

String netSsid() { return ssid; }
IPAddress netIp() { return up ? WiFi.softAPIP() : IPAddress(0, 0, 0, 0); }
int netClients() { return up ? WiFi.softAPgetStationNum() : 0; }
