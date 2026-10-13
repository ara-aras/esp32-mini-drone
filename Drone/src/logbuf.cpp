#include "logbuf.h"
#include <stdarg.h>

namespace {
const uint32_t CAP = 6144;  // timestamps cost ~11 chars per line
char ring[CAP];
uint32_t total = 0;
uint32_t serialCursor = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
char outBuf[CAP + 1];  // only used from the loop task

// Copies ring bytes [cursor, total) into outBuf, advances cursor. Returns length.
uint32_t drain(uint32_t &cursor) {
  uint32_t len = 0;
  portENTER_CRITICAL(&mux);
  uint32_t t = total;
  uint32_t oldest = t > CAP ? t - CAP : 0;
  if (cursor > t || cursor < oldest) cursor = oldest;  // reboot or overflow
  for (uint32_t i = cursor; i < t; i++) outBuf[len++] = ring[i % CAP];
  cursor = t;
  portEXIT_CRITICAL(&mux);
  outBuf[len] = 0;
  return len;
}
}  // namespace

void dlog(const char *fmt, ...) {
  char tmp[256];
  // Timestamp at the start of every line so events can be lined up with the
  // blackbox. Multi-line messages get one per line; partial lines continue.
  uint32_t ms = millis();
  int p = snprintf(tmp, sizeof(tmp), "[%4lu.%03lu] ", (unsigned long)(ms / 1000), (unsigned long)(ms % 1000));
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(tmp + p, sizeof(tmp) - p, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof(tmp) - p) n = sizeof(tmp) - 1 - p;
  const char *msg = tmp + p;
  portENTER_CRITICAL(&mux);
  bool lineStart = total == 0 || ring[(total - 1) % CAP] == '\n';
  for (int i = 0; i < n; i++) {
    if (lineStart) {
      for (int k = 0; k < p; k++) ring[(total + k) % CAP] = tmp[k];
      total += p;
      lineStart = false;
    }
    ring[total % CAP] = msg[i];
    total++;
    if (msg[i] == '\n') lineStart = true;
  }
  portEXIT_CRITICAL(&mux);
}

void logFlushSerial() {
  uint32_t n = drain(serialCursor);
  if (n) Serial.write((const uint8_t *)outBuf, n);
}

String logSince(uint32_t &cursor) {
  drain(cursor);
  return String(outBuf);
}
