#pragma once
// CRTP-over-UDP framing used by the ESP-Drone app (host-testable, header only).
// UDP datagram = [header][payload...][checksum], checksum = sum of all previous bytes.
// header = port<<4 | channel (bits 2-3 reserved, ignored). Floats are little-endian,
// which matches the ESP32 and x86, so memcpy is correct.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace crtp {

constexpr uint8_t PORT_CONSOLE = 0;
constexpr uint8_t PORT_COMMANDER = 3;
constexpr uint8_t PORT_GENERIC_SETPOINT = 7;
constexpr uint8_t PORT_LINK = 15;
constexpr size_t MAX_PAYLOAD = 31;

inline uint8_t checksum(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  while (n--) c = (uint8_t)(c + *d++);
  return c;
}

inline uint8_t header(uint8_t port, uint8_t channel) {
  return (uint8_t)(((port & 0x0F) << 4) | (channel & 0x03));
}

struct Frame {
  uint8_t port;
  uint8_t channel;
  const uint8_t *data;
  size_t len;
};

// Validates size and checksum. Payload excludes header and checksum.
inline bool parseUdp(const uint8_t *buf, size_t len, Frame &f) {
  if (!buf || len < 2 || len > MAX_PAYLOAD + 2) return false;
  if (checksum(buf, len - 1) != buf[len - 1]) return false;
  f.port = buf[0] >> 4;
  f.channel = buf[0] & 0x03;
  f.data = buf + 1;
  f.len = len - 2;
  return true;
}

// Legacy commander setpoint (port 3, channel 0): roll deg, pitch deg, yaw deg/s, thrust.
// The app already negates pitch before sending: positive pitch here = nose UP target.
struct Commander {
  float roll, pitch, yaw;
  uint16_t thrust;
};

inline bool parseCommander(const Frame &f, Commander &c) {
  if (f.port != PORT_COMMANDER || f.channel != 0 || f.len != 14) return false;
  memcpy(&c.roll, f.data, 4);
  memcpy(&c.pitch, f.data + 4, 4);
  memcpy(&c.yaw, f.data + 8, 4);
  memcpy(&c.thrust, f.data + 12, 2);
  return std::isfinite(c.roll) && std::isfinite(c.pitch) && std::isfinite(c.yaw);
}

// Wraps an arbitrary payload into a datagram (header + payload + checksum). Returns bytes written.
inline size_t buildPacket(uint8_t port, uint8_t channel, const uint8_t *payload, size_t n, uint8_t *out) {
  if (n > MAX_PAYLOAD) n = MAX_PAYLOAD;
  out[0] = header(port, channel);
  if (n) memcpy(out + 1, payload, n);
  out[1 + n] = checksum(out, 1 + n);
  return n + 2;
}

// Builds one console packet (port 0) carrying up to 30 chars. Returns bytes written.
inline size_t buildConsole(const char *text, size_t n, uint8_t *out) {
  if (n > 30) n = 30;
  out[0] = header(PORT_CONSOLE, 0);
  memcpy(out + 1, text, n);
  out[1 + n] = checksum(out, 1 + n);
  return n + 2;
}

}  // namespace crtp
