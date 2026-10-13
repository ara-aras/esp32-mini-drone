#include "blackbox.h"
#include "config.h"
#include "logbuf.h"
#include <LittleFS.h>

namespace {
enum class State : uint8_t { Idle, Recording, PostRoll, Frozen };

BbFrame ring[BLACKBOX_FRAMES];
volatile State state = State::Idle;
size_t head = 0;          // next write index
size_t count = 0;         // frames valid (<= BLACKBOX_FRAMES)
int decim = 0;
uint32_t postStart = 0;
char reason[40] = "";
bool fault = false;
volatile bool saveWanted = false;  // set on freeze after a fault
volatile bool saving = false;      // loop task is reading the ring
bool fsOk = false;

const char *FILE_BB = "/blackbox.bin";
const char *FILE_EV = "/events.txt";

struct __attribute__((packed)) FileHeader {
  uint32_t magic;      // 'B','B','O','X'
  uint16_t version;
  uint16_t frameSize;
  uint32_t count;
  uint32_t savedMs;
  char reason[40];
  uint8_t fault;
};
const uint32_t MAGIC = 0x584F4242u;

const BbFrame &frameAt(size_t i) {  // i = 0 oldest
  size_t start = (head + BLACKBOX_FRAMES - count) % BLACKBOX_FRAMES;
  return ring[(start + i) % BLACKBOX_FRAMES];
}

void saveToFlash() {
  if (!fsOk) { dlog("[bb] no filesystem, fault log kept in RAM only\n"); return; }
  uint32_t t0 = millis();
  File f = LittleFS.open(FILE_BB, "w");
  if (!f) { dlog("[bb] open %s failed\n", FILE_BB); return; }
  FileHeader h = {};
  h.magic = MAGIC; h.version = 1; h.frameSize = sizeof(BbFrame);
  h.count = count; h.savedMs = millis(); h.fault = fault;
  strncpy(h.reason, reason, sizeof(h.reason) - 1);
  bool ok = f.write((const uint8_t *)&h, sizeof(h)) == sizeof(h);
  for (size_t i = 0; ok && i < count; i++) {
    const BbFrame &fr = frameAt(i);
    ok = f.write((const uint8_t *)&fr, sizeof(fr)) == sizeof(fr);
  }
  f.close();
  // Event log next to it: the text around the fault is often the real clue.
  File e = LittleFS.open(FILE_EV, "w");
  if (e) {
    uint32_t cur = 0;
    String all = logSince(cur);
    e.print(all);
    e.close();
  }
  dlog("[bb] %s: %u frames + events saved to flash in %lu ms\n", ok ? "OK" : "WRITE FAILED",
       (unsigned)count, (unsigned long)(millis() - t0));
}
}  // namespace

// ================= flight-task side =================
void blackboxBegin() {
#if BLACKBOX_PERSIST
  uint32_t t0 = millis();
  fsOk = LittleFS.begin(true);  // true: format on first use (takes a few seconds once)
  if (fsOk) {
    dlog("[bb] LittleFS mounted in %lu ms, %u KB free%s\n", (unsigned long)(millis() - t0),
         (unsigned)((LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024),
         LittleFS.exists(FILE_BB) ? ", SAVED FAULT LOG PRESENT (bb saved)" : "");
  } else {
    dlog("[bb] LittleFS mount failed: fault logs will not survive a reboot\n");
  }
#endif
}

void blackboxArm() {
  if (saving) { dlog("[bb] busy saving, this flight is not recorded\n"); return; }
  head = 0; count = 0; decim = 0;
  reason[0] = 0; fault = false;
  state = State::Recording;
}

void blackboxDisarm(const char *why, bool isFault) {
  if (state != State::Recording) return;
  strncpy(reason, why, sizeof(reason) - 1);
  reason[sizeof(reason) - 1] = 0;
  fault = isFault;
  postStart = millis();
  state = State::PostRoll;
}

bool blackboxRecording() { return state == State::Recording || state == State::PostRoll; }

void blackboxRecord(const BbFrame &f) {
  if (!blackboxRecording()) return;
  if (++decim < LOOP_HZ / BLACKBOX_HZ) return;
  decim = 0;
  ring[head] = f;
  head = (head + 1) % BLACKBOX_FRAMES;
  if (count < BLACKBOX_FRAMES) count++;
  if (state == State::PostRoll && millis() - postStart >= BLACKBOX_POST_MS) {
    state = State::Frozen;
    if (fault) saveWanted = true;
  }
}

// ================= loop-task side =================
void blackboxLoop() {
  if (!saveWanted || state != State::Frozen) return;
  saveWanted = false;
  saving = true;
  saveToFlash();
  saving = false;
}

bool blackboxFrozen() { return state == State::Frozen; }
size_t blackboxCount() { return state == State::Frozen ? count : 0; }
const BbFrame &blackboxFrame(size_t i) { return frameAt(i); }
const char *blackboxReason() { return reason; }
bool blackboxSavedExists() { return fsOk && LittleFS.exists(FILE_BB); }
bool blackboxClearSaved() {
  if (!fsOk) return false;
  bool a = LittleFS.remove(FILE_BB);
  LittleFS.remove(FILE_EV);
  return a;
}

const char *blackboxCsvHeader() {
  return "ms,roll,pitch,gx,gy,gz,sp_roll,sp_pitch,sp_yaw,pid_roll,pid_pitch,pid_yaw,"
         "m1,m2,m3,m4,thr,rc_roll,rc_pitch,rc_yaw,vbat,armed,src,link";
}

size_t blackboxCsvLine(const BbFrame &f, char *buf, size_t len) {
  int n = snprintf(buf, len,
                   "%lu,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,"
                   "%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\n",
                   (unsigned long)f.ms, f.roll / 100.0f, f.pitch / 100.0f,
                   f.gx / 10.0f, f.gy / 10.0f, f.gz / 10.0f,
                   f.sp[0] / 10.0f, f.sp[1] / 10.0f, f.sp[2] / 10.0f,
                   f.pid[0] / 10000.0f, f.pid[1] / 10000.0f, f.pid[2] / 10000.0f,
                   f.m[0] / 255.0f, f.m[1] / 255.0f, f.m[2] / 255.0f, f.m[3] / 255.0f,
                   f.thr / 255.0f, f.rcR / 127.0f, f.rcP / 127.0f, f.rcY / 127.0f,
                   f.vbat / 40.0f, f.flags & 1, (f.flags >> 1) & 3, (f.flags >> 3) & 1);
  return n < 0 ? 0 : (size_t)n;
}

bool blackboxDump(bool saved, void (*sink)(const char *, size_t, void *), void *ctx) {
  char line[256];
  if (saved) {
    if (!blackboxSavedExists()) return false;
    File f = LittleFS.open(FILE_BB, "r");
    if (!f) return false;
    FileHeader h;
    if (f.read((uint8_t *)&h, sizeof(h)) != sizeof(h) || h.magic != MAGIC ||
        h.frameSize != sizeof(BbFrame)) { f.close(); return false; }
    int n = snprintf(line, sizeof(line), "# saved fault log, reason: %s, %u frames, saved at %lu ms\n%s\n",
                     h.reason, (unsigned)h.count, (unsigned long)h.savedMs, blackboxCsvHeader());
    sink(line, n, ctx);
    BbFrame fr;
    while (f.read((uint8_t *)&fr, sizeof(fr)) == sizeof(fr)) sink(line, blackboxCsvLine(fr, line, sizeof(line)), ctx);
    f.close();
    File e = LittleFS.open(FILE_EV, "r");
    if (e) {
      sink("# events:\n", 10, ctx);
      while (e.available()) {  // '#' prefix keeps the CSV parseable
        size_t n2 = e.readBytesUntil('\n', line, sizeof(line) - 2);
        line[n2++] = '\n';
        sink("# ", 2, ctx);
        sink(line, n2, ctx);
      }
      e.close();
    }
    return true;
  }
  if (!blackboxFrozen()) return false;
  int n = snprintf(line, sizeof(line), "# reason: %s (%s), %u frames\n%s\n", reason,
                   fault ? "FAULT" : "normal", (unsigned)count, blackboxCsvHeader());
  sink(line, n, ctx);
  for (size_t i = 0; i < count; i++) sink(line, blackboxCsvLine(frameAt(i), line, sizeof(line)), ctx);
  return true;
}
