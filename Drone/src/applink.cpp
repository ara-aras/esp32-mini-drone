#include "applink.h"
#include "config.h"

#if APP_LINK_ENABLED
#include <AsyncUDP.h>
#include <atomic>
#include "crtp.h"
#include "radio.h"
#include "logbuf.h"

namespace {
AsyncUDP udp;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
IPAddress remoteIp;
uint16_t remotePort = 0;
std::atomic<uint32_t> badPackets{0};
uint32_t seenPorts = 0;
char pending[256];
size_t pendingLen = 0;
uint8_t echoBuf[crtp::MAX_PAYLOAD];   // last link echo request, replayed from the loop task
size_t echoLen = 0;
bool echoPending = false;

void onPacket(AsyncUDPPacket &pkt) {
  crtp::Frame f;
  if (!crtp::parseUdp(pkt.data(), pkt.length(), f)) {
    badPackets++;
    return;
  }
  portENTER_CRITICAL(&mux);
  remoteIp = pkt.remoteIP();
  remotePort = pkt.remotePort();
  portEXIT_CRITICAL(&mux);

  crtp::Commander c;
  if (crtp::parseCommander(f, c)) {
    radioFeedApp(c.roll, c.pitch, c.yaw, c.thrust);
    return;
  }
  if (f.port == crtp::PORT_LINK) {  // ch 0 = echo request (Crazyflie clients ping with it)
    if (f.channel == 0) {
      portENTER_CRITICAL(&mux);
      echoLen = f.len; memcpy(echoBuf, f.data, f.len); echoPending = true;
      portEXIT_CRITICAL(&mux);
    }
    return;
  }
  uint32_t bit = 1UL << f.port;
  if (!(seenPorts & bit)) {
    seenPorts |= bit;
    dlog("[app] ignoring CRTP port %u ch %u (%s)\n", f.port, f.channel,
         f.port == crtp::PORT_GENERIC_SETPOINT ? "height-hold mode is not supported - turn it off in the app"
                                               : "not implemented");
  }
}
}  // namespace

bool appLinkBegin() {
  if (!udp.listen(APP_UDP_PORT)) {
    dlog("[app] UDP listen on %d failed\n", APP_UDP_PORT);
    return false;
  }
  udp.onPacket([](AsyncUDPPacket &pkt) { onPacket(pkt); });
  dlog("[app] ESP-Drone app link listening on UDP %d\n", APP_UDP_PORT);
  return true;
}

void appLinkConsole(const char *s) {
  size_t n = strlen(s);
  portENTER_CRITICAL(&mux);
  if (pendingLen + n < sizeof(pending)) {
    memcpy(pending + pendingLen, s, n);
    pendingLen += n;
  }
  portEXIT_CRITICAL(&mux);
}

void appLinkLoop() {
  char buf[sizeof(pending)];
  size_t n;
  IPAddress ip;
  uint16_t port;
  uint8_t echo[crtp::MAX_PAYLOAD];
  size_t echoN = 0;
  portENTER_CRITICAL(&mux);
  n = pendingLen;
  memcpy(buf, pending, n);
  pendingLen = 0;
  ip = remoteIp;
  port = remotePort;
  if (echoPending) { echoN = echoLen; memcpy(echo, echoBuf, echoLen); echoPending = false; echoN++; }
  portEXIT_CRITICAL(&mux);
  if (!port) return;
  uint8_t out[crtp::MAX_PAYLOAD + 2];
  for (size_t off = 0; off < n; off += 30) {
    size_t len = crtp::buildConsole(buf + off, n - off, out);
    udp.writeTo(out, len, ip, port);
  }
  if (echoN) {  // echo reply: same payload back on port 15 ch 0
    udp.writeTo(out, crtp::buildPacket(crtp::PORT_LINK, 0, echo, echoN - 1, out), ip, port);
  }
}

uint32_t appLinkBadPackets() { return badPackets; }

#else
bool appLinkBegin() { return true; }
void appLinkLoop() {}
void appLinkConsole(const char *) {}
uint32_t appLinkBadPackets() { return 0; }
#endif
