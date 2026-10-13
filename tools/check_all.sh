#!/usr/bin/env bash
# One-shot verification: host unit tests + firmware compile for both sketches.
# Needs: g++, make, arduino-cli with the esp32 core installed
#   arduino-cli config add board_manager.additional_urls \
#     https://espressif.github.io/arduino-esp32/package_esp32_index.json
#   arduino-cli core update-index && arduino-cli core install esp32:esp32
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FQBN="${FQBN:-esp32:esp32:esp32}"

echo "== unit tests =="
make -C "$ROOT/tests" --no-print-directory

if ! command -v arduino-cli >/dev/null; then
  echo "arduino-cli not found - skipping firmware compile"; exit 0
fi
for s in Drone Transmitter; do
  echo "== compile $s ($FQBN) =="
  arduino-cli compile -b "$FQBN" --warnings all "$ROOT/$s"
done
# Optional second core (2.x) in its own data dir (see check_all.bat for install).
V2="${ARDUINO_V2_DATA:-$LOCALAPPDATA/Arduino15-v2}"
if [ -d "$V2/packages/esp32" ]; then
  for s in Drone Transmitter; do
    echo "== compile $s (core 2.x) =="
    ARDUINO_DIRECTORIES_DATA="$V2" arduino-cli compile -b "$FQBN" --warnings all "$ROOT/$s"
  done
fi
echo "== ALL CHECKS PASSED =="
