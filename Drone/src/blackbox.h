#pragma once
#include <Arduino.h>

// Flight recorder ("blackbox"). The flight task appends compact frames at
// BLACKBOX_HZ while armed; after a disarm it keeps going for BLACKBOX_POST_MS
// and then freezes, so the last ~14 s before AND 1 s after the event survive.
// A fault disarm (failsafe, crash tilt, IMU, watchdog) is also saved to flash
// by the loop task, because a crash usually disconnects the battery.
// Dump as CSV: web /api/blackbox (or ?saved=1), serial `bb` / `bb saved`.

struct __attribute__((packed)) BbFrame {
  uint32_t ms;
  int16_t  roll, pitch;   // deg * 100
  int16_t  gx, gy, gz;    // deg/s * 10
  int16_t  sp[3];         // rate setpoints, deg/s * 10
  int16_t  pid[3];        // PID outputs * 10000
  uint8_t  m[4];          // motor 0..255
  uint8_t  thr;           // throttle after soft-start cap, 0..255
  int8_t   rcR, rcP, rcY; // sticks * 127
  uint8_t  vbat;          // volts * 40
  uint8_t  flags;         // bit0 armed, bit1-2 RcSource, bit3 link ok
};

// ---- flight-task side ----
void blackboxBegin();                         // mounts LittleFS (formats on first use)
void blackboxArm();                           // clear and start recording
void blackboxDisarm(const char *reason, bool fault);  // post-roll, then freeze
bool blackboxRecording();                     // cheap check before building a frame
void blackboxRecord(const BbFrame &f);        // call every loop; decimates itself

// ---- loop-task side ----
void blackboxLoop();                          // saves a frozen fault log to flash
bool blackboxFrozen();
size_t blackboxCount();                       // frames in RAM (only valid when frozen)
const BbFrame &blackboxFrame(size_t i);       // oldest first
const char *blackboxReason();
bool blackboxSavedExists();
bool blackboxClearSaved();
// CSV helpers. Header has no trailing newline; line writes into buf.
const char *blackboxCsvHeader();
size_t blackboxCsvLine(const BbFrame &f, char *buf, size_t len);
// Streams a CSV dump (RAM or saved copy) through `sink`, one chunk at a time.
// Returns false if nothing is available. Loop task only.
bool blackboxDump(bool saved, void (*sink)(const char *, size_t, void *), void *ctx);
