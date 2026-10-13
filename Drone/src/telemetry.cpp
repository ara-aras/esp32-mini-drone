#include "telemetry.h"

namespace {
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Telemetry cur = {};
}

void telemetryPublish(const Telemetry &t) {
  portENTER_CRITICAL(&mux);
  cur = t;
  portEXIT_CRITICAL(&mux);
}

Telemetry telemetryGet() {
  portENTER_CRITICAL(&mux);
  Telemetry t = cur;
  portEXIT_CRITICAL(&mux);
  return t;
}
