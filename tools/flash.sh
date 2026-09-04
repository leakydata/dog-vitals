#!/usr/bin/env bash
# Build and flash the dog-vitals firmware to the XIAO nRF52840 Sense.
#
# There is no SWD probe on this bench, so the only route in is the Adafruit
# bootloader. Two things about it are worth knowing before changing anything
# here, both learned the hard way on the Boswell board:
#
#   * Writing the UF2 to the mass-storage drive is accepted, resets the board,
#     and leaves it sitting back in the bootloader having flashed nothing --
#     with no error anywhere. Serial DFU is the path that actually works, so it
#     is the only one used.
#   * Which /dev/ttyACM* is the DFU port is not fixed. The board exposes more
#     than one CDC interface and the numbering moves between reflashes, so
#     every port is tried rather than assuming ACM0.
set -uo pipefail

cd "$(dirname "$0")/.."
FQBN="Seeeduino:nrf52:xiaonRF52840Sense"
SKETCH="firmware/dog_vitals_imu"
BUILD="build"

echo "==> building"
arduino-cli compile --fqbn "$FQBN" --output-dir "$BUILD" "$SKETCH" || exit 1
ZIP="$BUILD/dog_vitals_imu.ino.zip"
[ -f "$ZIP" ] || { echo "no DFU package produced" >&2; exit 1; }

# Refuse to flash something older than the code it claims to be. A chained
# build-then-flash once pushed a stale image after a compile error and the next
# hour was spent testing firmware that did not contain the change under test.
NEWER=$(find "$SKETCH" -newer "$ZIP" \( -name '*.ino' -o -name '*.cpp' -o -name '*.h' \) -print -quit)
[ -z "$NEWER" ] || { echo "package is older than $NEWER" >&2; exit 1; }

in_bootloader() { lsusb | grep -q "2886:0045"; }
in_app()        { lsusb | grep -q "2886:8045"; }

echo "==> entering bootloader"
if in_bootloader; then
  echo "    already there"
else
  # Ask whatever is running to reboot itself. Three dialects, because the board
  # may be running this firmware ('d' on its console), the older Zephyr build
  # (a shell command), or something that answers to neither.
  for p in /dev/ttyACM*; do
    [ -e "$p" ] || continue
    (stty -F "$p" raw -echo 115200 2>/dev/null &&
     exec 3<>"$p" &&
     printf 'd\r\nboswell dfu\r\n' >&3 &&
     exec 3<&-) 2>/dev/null
  done
  sleep 2

  # The 1200-baud open-and-close is the Arduino IDE's convention and the
  # bootloader honours it whatever is running above.
  if ! in_bootloader; then
    for p in /dev/ttyACM*; do
      [ -e "$p" ] || continue
      python3 -c "
import serial,sys
try:
    s=serial.Serial('$p',1200); s.dtr=False; s.close()
except Exception: pass
" 2>/dev/null
    done
  fi

  for _ in $(seq 1 20); do in_bootloader && break; sleep 1; done
fi

in_bootloader || {
  echo "board is not in the bootloader -- double-tap RESET and rerun" >&2
  exit 1
}
sleep 1

echo "==> flashing over serial DFU"
FLASHED=0
for port in /dev/ttyACM*; do
  [ -e "$port" ] || continue
  if adafruit-nrfutil dfu serial -pkg "$ZIP" -p "$port" -b 115200 --singlebank 2>&1 \
       | tail -3 | grep -q "Device programmed"; then
    echo "    programmed via $port"
    FLASHED=1
    break
  fi
done
[ "$FLASHED" = 1 ] || { echo "serial DFU failed on every port" >&2; exit 1; }

# The bootloader enumerates as 2886:0045 and the application as 2886:8045.
# Without this check a flash that silently did nothing looks like a success.
echo "==> verifying the application is running"
for _ in $(seq 1 25); do
  in_app && { echo "    running (2886:8045)"; exit 0; }
  sleep 1
done
echo "still in the bootloader -- the application did not start" >&2
exit 1
