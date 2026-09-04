# Dog Vitals — IMU streaming for respiration and heart rate

Raw 6-axis IMU streaming from a Seeed XIAO nRF52840 Sense over BLE, plus a
browser tool for plotting, measuring and reviewing it. The point is to find out
*whether* and *how* breathing and heart rate can be read from an IMU taped to a
neck, and then from one on a dog collar — so the device ships raw samples with
trustworthy timing and does no interpretation of its own.

## Status

Working and measured on the attached board:

| | |
|---|---|
| Stream rate | 207.9 Hz sustained over BLE, 0 dropped samples, 0 FIFO overruns, 0 notify drops in a 20 s run |
| Measured ODR | 207.88 Hz against the device clock (nominal 208) |
| Packet | 240 B = 12 B header + 19 samples × 12 B, ATT MTU 247 |
| Resolution | ±2 g at 0.061 mg/LSB, ±245 °/s at 0.00875 °/s/LSB |
| Battery | 4.09 V read on-board through the P0.14-gated divider |

The analysis chain reads 15.00 breaths/min and 72.01 bpm from a synthetic
recording built with exactly those rates, with the independent peak-count
method agreeing to 72.00 and an interval SD of 2.3 ms.

## Quick start

```bash
./tools/flash.sh          # build and flash the firmware
./tools/serve.sh          # then open http://localhost:8420/
```

**The browser does not do Bluetooth.** `tools/server.py` holds the BLE
connection and forwards the device's own notification bytes to the page over a
WebSocket, so any browser works — no Web Bluetooth, no permission prompt, no
device picker, and the stream survives a page reload because the radio link
belongs to the server rather than to the tab. It reconnects to the board on its
own if the link drops.

The page still falls back to Web Bluetooth if it is served statically without
the bridge, and the two share their decoding and statistics code, so nothing
downstream can tell them apart.

Streaming starts when something subscribes and stops when nothing is listening,
so an unattended board is not burning battery on the radio.

**Record** captures to IndexedDB in the browser *and* writes a CSV to `data/`
server-side — the file on disk is the one that survives a cleared profile and
that other tools can read.

## Layout

```
firmware/dog_vitals_imu/    Arduino sketch (Seeeduino:nrf52:xiaonRF52840Sense)
  dog_vitals_imu.ino        BLE service, FIFO drain, USB console
  lsm6ds3.{h,cpp}           direct-register IMU driver
  protocol.h                wire format — mirrored by web/lib/protocol.js
  i2cdiag.h                 bit-banged bus probe, used by the 'i' command
web/                        the browser tool (no build step, plain ES modules)
  lib/protocol.js           packet and status decoding
  lib/ble.js                Web Bluetooth link + loss accounting
  lib/dsp.js                filters, autocorrelation rate, spectra
  lib/plot.js               canvas strip charts and spectrum
  lib/store.js              buffering, recording, IndexedDB, CSV/JSON
tools/flash.sh              build + serial DFU + verify
tools/server.py             BLE bridge + static server (the thing you run)
tools/serve.sh              one-line wrapper for server.py
tools/capture_neck.py       headless capture, triggered by picking the board up
tools/ble_probe.py          headless capture and link statistics
tools/make_test_csv.py      synthetic session with a known answer
```

## Measuring vitals

Both rates come from **autocorrelation over the visible window**, not from
counting the marks drawn on the plot. Counting is fragile — one missed or
doubled peak moves the answer by a large fraction — while autocorrelation uses
every cycle at once and reports, as its peak height, how periodic the window
actually was. That number drives the confidence bar. The peak marks are still
drawn, because seeing whether the thing being counted *looks* like a breath or
a beat is the point of the exercise.

**Breathing** comes from the accelerometer, projected onto its principal axis
so that no calibration step or "which way up is the collar" decision is needed.
It is band-passed with a difference of two moving averages rather than a
biquad: at 208 Hz a 0.1 Hz corner is fc/fs = 0.0005, where a direct-form biquad
is numerically fragile on exactly the signal it is meant to pass.

**Heart rate** comes from the gyroscope by default. At the neck the carotid
pulse is a rotation of the skin surface more than a translation, so the gyro
often sees it better than the accelerometer — but both are selectable, and
which one wins is one of the things to find out. The band-passed trace
oscillates at a mechanical resonance; it is the *bursts* that repeat once per
beat, so the rate is taken from the envelope with its DC removed.

**The spectrum view is the actual instrument here.** Filter corners taken from
a textbook are a starting guess; the spectrum, in dB so the millig-scale
cardiac content and the tens-of-millig breathing are visible at once, is the
evidence for where the signal really sits on this body, at this mounting point,
through this much tape.

### Presets

| | Breathing band | Cardiac band | HR search |
|---|---|---|---|
| Human — neck | 0.10–0.60 Hz | 4–25 Hz | 40–150 bpm |
| Dog — resting | 0.15–0.90 Hz | 4–30 Hz | 50–180 bpm |
| Dog — panting | 1.0–5.0 Hz | 8–30 Hz | 60–200 bpm |

A panting dog breathes at up to 5 Hz, which runs into the bottom of the cardiac
band — hence the higher cardiac high-pass on that preset. Expect panting to be
the hard case.

## Honest data

Three things are deliberately visible rather than smoothed over, because a rate
computed from data that is quietly wrong is worse than no rate at all:

- **The sample index is the time base**, not packet arrival. BLE delivers in
  bursts, and tens of milliseconds of jitter is indistinguishable from real
  motion at breathing frequencies. Sample *k* of a packet happened at
  `(idx + k) / odr` seconds, full stop.
- **Gaps stay gaps.** If the device loses samples to a FIFO overrun, the
  firmware advances the index by the elapsed device time, so the hole shows up
  in the index instead of time being silently compressed. The UI interpolates
  across short gaps to keep the filters fed, counts the invented samples, and
  says so; past 1.5 s it restarts the window instead.
- **The ODR is measured, not assumed.** The LSM6DS3's oscillator is trimmed to
  a few percent and drifts with temperature, and a 2% error there is a 2% error
  in every rate derived from it. The device stamps each packet with its own
  clock and the host solves for the real rate.

## When not to believe a rate

Sleeping respiratory rate is the clinical reason this device exists — vets read
a sustained figure across a night as an early sign of heart failure — so the way
an estimate fails matters more than how often it succeeds.

It fails in a specific and dangerous way. Given a window with no real
periodicity, autocorrelation settles wherever the search happens to peak, and
that is very often the top of the band: 0.60 Hz reads as **36.3 breaths/min**.
That is not obvious nonsense anyone would discard. It lands squarely in the
range that signals trouble, so a restless minute looks exactly like the
condition the device is watching for.

`dsp.gateRate()` therefore refuses an estimate that is

- **pinned to either band edge** — within 4% of the search limits, which is the
  filter talking rather than the subject;
- **below 0.4 autocorrelation confidence** — the peak height already says how
  periodic the window was;
- **taken while the subject moved** — the accelerometer measures this directly,
  so there is no need to infer it.

`dsp.acceptedMedian()` then aggregates what survives, because the clinical
figure is a trend across a night rather than any single window, and it reports
coverage alongside the median: a confident number computed from 3% of the night
is not a measurement, and the coverage figure is what says so.

`tools/gate_check.mjs` replays recorded sessions through the same analysis the
page runs, so the gate can be checked against real failures rather than
synthetic ones. Over six neck recordings it rejects both sessions that produced
36.3 br/min, takes the one with sustained movement to zero coverage, and keeps
100% of the stillest session at a median of 9.3 br/min.

One caveat the same run exposed: the human preset's 0.10 Hz floor is 6.0
br/min, which is close enough to a slow adult's actual rate that genuine
readings get rejected as pinned to the bottom. That is the gate being honest
about a measurement at the edge of its range. The `dogRest` preset starts at
0.15 Hz against a sleeping dog's 15–30 br/min, so it has the headroom the human
one lacks.

## Recording

**Start recording** captures at full resolution into memory and saves to
IndexedDB on stop. Sessions can be replayed in **Review** mode — same code path
as live, so a recording cannot disagree with what was on screen — and exported
as CSV or JSON. Recorded values are raw counts with the scale factors in the
metadata, so a session stays a record rather than a derivation.

CSV import accepts both the exported format and the firmware's own USB console
output.

## USB console

`screen /dev/ttyACM0 115200`, or any terminal. Single keys:

| Key | Does |
|---|---|
| `s` | status: IMU, WHO_AM_I, rates, MTU, samples/packet, overruns, battery |
| `c` | toggle CSV streaming (raw counts; scales printed in the header) |
| `r` | pause/resume sampling |
| `i` | I2C bus diagnostic — rail levels, pin survey, bit-banged scan |
| `d` | reboot into serial DFU |
| `u` | reboot into the UF2 bootloader |

The USB path is independent of BLE, so it works with no browser connected. It
does not quite keep up at 208 Hz — formatting is the bottleneck, and the run
will log some overruns — but the gaps are marked honestly in the index.

## The hardware

One Seeed XIAO nRF52840 Sense — nRF52840 SoC with an LSM6DS3TR-C 6-axis IMU on
board — with an LP502030 lithium pouch cell soldered to the back. The cell is
5 × 20 × 30 mm, so the pair stack into a small flat rectangle roughly the
footprint of the battery itself.

Right now the two are twist-tied together, which is honest about what this
stage is: a bench rig for finding out whether the signal exists at all, not
something to put on an animal. It reads a real human heart rate and breathing
rate through skin at the neck, which is what it was for. A printed enclosure
that mounts to a collar is the next physical step, and until that exists none
of the measurements here have been taken through fur — which is the single
biggest unknown between this and a working dog monitor.

Nothing about the approach is specific to this board. It needs a 6-axis IMU
that can sustain a couple of hundred hertz, a radio, and a battery; the
firmware is the only part that knows which one it is.

## Two hardware traps on this board

Both cost real time; both are one line to avoid.

**The IMU supply pin needs high drive.** It is P1.08 (Arduino D15) and must be
configured `H0H1`. A plain `pinMode(OUTPUT)` cannot source enough current and
the sensor never starts. Because the bus pull-ups sit on that same switched
rail, the symptom is that SCL, SDA *and* INT1 all read low too — it looks like
four shorted pins or a dead board, not a supply problem. Nothing in the board
variant header hints at it. (Zephyr's port of this board carries the same fix
as `NRF_GPIO_DRIVE_S0H1` in its devicetree.)

**This core's I2C driver spins with no timeout.** `Wire_nRF52.cpp` ends a read
with `while(!_p_twim->EVENTS_STOPPED);` — no error escape. A peripheral that
does not answer does not produce a failed call, it produces firmware that never
reaches `loop()`: no console, no advertising, nothing, indistinguishable from a
bad flash. So `setup()` here brings up the radio and nothing else — the
SoftDevice keeps advertising on its own even if the app thread wedges later, so
"is it advertising?" survives a hang — and the sensor and ADC are brought up
from `loop()` with a retry, behind a check that both bus lines are idle-high.

## Flashing

`tools/flash.sh` builds, gets the board into the bootloader (the `d` console
command, then the 1200-baud touch as a fallback), flashes over **serial DFU**,
and verifies the application came back up. Writing the UF2 to the mass-storage
drive is *not* used: it is accepted, resets the board, and flashes nothing, with
no error anywhere.

There is no SWD probe on this bench, so the bootloader is the only way in. If a
flash ever leaves the board unresponsive, double-tap RESET to force the
bootloader and rerun.

## Next

- On-board logging to the 2 MB QSPI flash, for a dog that walks out of range.
  Live streaming is the right tool for working out what the signal looks like;
  it is the wrong one for a collar.
- Motion rejection. A walking dog will swamp both bands; the gyro magnitude is
  the obvious gate for "this window is not measurable".
- Once the bands are settled, move the rate estimation on-device and stream
  numbers instead of samples, which is where the battery win is.

## Licence

MIT — see `LICENSE`. Use it for anything, commercial included; the only
condition is that the copyright notice travels with it.

This is arithmetic applied to accelerometer samples. Bandpass filters and
autocorrelation are not anyone's property, and it would be strange to pretend
otherwise. If it turns out to be useful, a mention of the project is welcome
and is not required.

Not a veterinary device. It is a research instrument for finding out what an
IMU can and cannot see, and the whole point of the gating above is that it
tells you when it is not measuring.
