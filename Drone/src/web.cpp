#include "web.h"
#include "config.h"

#if WEB_ENABLED
#include <WiFi.h>
#include <WebServer.h>
#include "web_page.h"
#include "telemetry.h"
#include "commands.h"
#include "logbuf.h"
#include "settings.h"
#include "battery.h"
#include "bench.h"
#include "net.h"
#include "radio.h"
#include "blackbox.h"

namespace {
WebServer server(80);
bool started = false;

void handleRoot() { server.send(200, "text/html", WEB_PAGE); }

void handleTelem() {
  Telemetry t = telemetryGet();
  char j[640];
  snprintf(j, sizeof(j),
           "{\"roll\":%.2f,\"pitch\":%.2f,\"g\":[%.1f,%.1f,%.1f],\"sp\":[%.1f,%.1f,%.1f],"
           "\"pid\":[%.4f,%.4f,%.4f],\"m\":[%.3f,%.3f,%.3f,%.3f],\"thr\":%.3f,"
           "\"armed\":%d,\"bench\":%d,\"rxAge\":%lu,\"rxN\":%lu,\"imuErr\":%lu,"
           "\"hz\":%.0f,\"workUs\":%lu,\"stack\":%lu,\"vbat\":%.2f,\"unlock\":%lu,\"src\":\"%s\",\"clients\":%d,\"bb\":%d,\"bbSaved\":%d}",
           t.roll, t.pitch, t.gx, t.gy, t.gz, t.sp[0], t.sp[1], t.sp[2],
           t.pid[0], t.pid[1], t.pid[2], t.motor[0], t.motor[1], t.motor[2], t.motor[3],
           t.throttle, t.armed, t.benchBusy,
           (unsigned long)(t.rxAge > 99999 ? 99999 : t.rxAge), (unsigned long)t.rxPackets,
           (unsigned long)t.imuErrors, t.loopHz, (unsigned long)t.workUs, (unsigned long)t.stackFree,
           batteryVoltage(),
           (unsigned long)(benchUnlockRemainingMs() / 1000), rcSourceName((RcSource)t.rcSource), netClients(),
           blackboxFrozen() ? 1 : 0, blackboxSavedExists() ? 1 : 0);
  server.send(200, "application/json", j);
}

void handleLog() {
  uint32_t cursor = (uint32_t)strtoul(server.arg("since").c_str(), nullptr, 10);
  String text = logSince(cursor);
  server.sendHeader("X-Cursor", String(cursor));
  server.send(200, "text/plain", text);
}

void handleCmd() {
  String line = server.hasArg("plain") ? server.arg("plain") : server.arg("c");
  cmdExecute(line, telemetryGet().armed);
  server.send(200, "text/plain", "ok");
}

void handleSettings() { server.send(200, "application/json", settingsJson()); }

void handleBlackbox() {
  bool saved = server.arg("saved") == "1";
  if (!(saved ? blackboxSavedExists() : blackboxFrozen())) {
    server.send(404, "text/plain", saved ? "no saved fault log" : "nothing recorded (arm, fly, disarm, wait 1 s)");
    return;
  }
  server.sendHeader("Content-Disposition", saved ? "attachment; filename=blackbox_fault.csv"
                                                 : "attachment; filename=blackbox.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  // Batch small lines into ~1 KB chunks: each sendContent is one TCP write.
  struct Acc { char buf[1024]; size_t n; } acc{{0}, 0};
  blackboxDump(saved, [](const char *d, size_t n, void *ctx) {
    Acc *a = (Acc *)ctx;
    if (a->n + n > sizeof(a->buf)) { server.sendContent(a->buf, a->n); a->n = 0; }
    if (n > sizeof(a->buf)) { server.sendContent(d, n); return; }
    memcpy(a->buf + a->n, d, n);
    a->n += n;
  }, &acc);
  if (acc.n) server.sendContent(acc.buf, acc.n);
  server.sendContent("");  // end of chunked body
}
}  // namespace

void webBegin() {
  server.on("/", handleRoot);
  server.on("/api/telem", handleTelem);
  server.on("/api/log", handleLog);
  server.on("/api/cmd", HTTP_POST, handleCmd);
  server.on("/api/settings", handleSettings);
  server.on("/api/blackbox", handleBlackbox);
  server.onNotFound([] { server.send(404, "text/plain", "not found"); });
  server.begin();
  started = true;
  dlog("[web] dashboard: http://%s\n", netIp().toString().c_str());
}

void webLoop() {
  if (started) server.handleClient();
}

#else
void webBegin() {}
void webLoop() {}
#endif
