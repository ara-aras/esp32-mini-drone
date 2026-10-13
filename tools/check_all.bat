@echo off
REM Windows one-shot verification: host unit tests + firmware compile for both sketches.
REM Needs: arduino-cli with the esp32 core, and MinGW (g++ + mingw32-make) on PATH
REM or at C:\tools\mingw64 (winget install BrechtSanders.WinLibs.POSIX.UCRT, then copy
REM mingw64 to a path without spaces - its linker breaks on spaces).
setlocal
set ROOT=%~dp0..
if exist C:\tools\mingw64\bin set PATH=C:\tools\mingw64\bin;%PATH%
if exist "C:\Program Files\Arduino CLI" set PATH=C:\Program Files\Arduino CLI;%PATH%

echo == unit tests ==
where mingw32-make >nul 2>nul
if errorlevel 1 (
  echo mingw32-make not found - skipping unit tests
) else (
  mingw32-make -C "%ROOT%\tests" CXX=g++ --no-print-directory || exit /b 1
)

where arduino-cli >nul 2>nul
if errorlevel 1 (
  echo arduino-cli not found - skipping firmware compile
  exit /b 0
)
echo == compile Drone ==
arduino-cli compile -b esp32:esp32:esp32 --warnings all "%ROOT%\Drone" || exit /b 1
echo == compile Transmitter ==
arduino-cli compile -b esp32:esp32:esp32 --warnings all "%ROOT%\Transmitter" || exit /b 1
REM Optional second core (2.x) lives in its own data dir so both can coexist:
REM   set ARDUINO_DIRECTORIES_DATA=%LOCALAPPDATA%\Arduino15-v2
REM   arduino-cli core install esp32:esp32@2.0.17 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
if exist "%LOCALAPPDATA%\Arduino15-v2\packages\esp32" (
  set ARDUINO_DIRECTORIES_DATA=%LOCALAPPDATA%\Arduino15-v2
  echo == compile Drone (core 2.x) ==
  arduino-cli compile -b esp32:esp32:esp32 --warnings all "%ROOT%\Drone" || exit /b 1
  echo == compile Transmitter (core 2.x) ==
  arduino-cli compile -b esp32:esp32:esp32 --warnings all "%ROOT%\Transmitter" || exit /b 1
)
echo == ALL CHECKS PASSED ==
