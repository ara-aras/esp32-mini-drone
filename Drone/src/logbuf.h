#pragma once
#include <Arduino.h>

// Log to a ring buffer read by Serial (via logFlushSerial) and the web dashboard.
// Safe to call from any task and never blocks: the flight task must not wait on UART.
void dlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Writes everything logged since the last call to Serial. Call from the loop task.
void logFlushSerial();

// Returns text logged since `cursor`, and advances `cursor`.
String logSince(uint32_t &cursor);
