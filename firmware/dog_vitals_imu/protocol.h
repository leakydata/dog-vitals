// Wire format shared by the firmware and the web client. Everything is
// little-endian, which is the nRF52's native order and also what a JS
// DataView defaults to when told so explicitly.
//
// Keep this in sync with web/lib/protocol.js -- the two describe the same
// bytes and there is no schema negotiation beyond the version byte.
#pragma once
#include <stdint.h>

#define PROTO_VERSION 1

// (247 MTU - 3 ATT header - 12 packet header) / 12 bytes per sample
#define MAX_SAMPLES_PER_PKT 19

// Notified on the data characteristic, followed immediately by `n` samples of
// six int16 each in the sensor's own FIFO order: gx, gy, gz, ax, ay, az.
//
// `idx` is the authoritative time base. Sample k of a packet occurred at
// (idx + k) / odr seconds after the stream started, regardless of when the
// packet arrived -- BLE delivery jitter of tens of milliseconds would
// otherwise be indistinguishable from real motion at breathing frequencies.
// `t_ms` is the host-side clock reference used to measure the sensor's actual
// ODR, which is trimmed to a few percent and drifts with temperature.
struct __attribute__((packed)) PktHeader {
  uint8_t  ver;
  uint8_t  n;
  uint16_t seq;
  uint32_t idx;
  uint32_t t_ms;
};

// Read or notified on the status characteristic. Carries the scale factors so
// the client never has to hard-code a range it did not choose.
struct __attribute__((packed)) StatusPkt {
  uint8_t  ver;
  uint8_t  flags;          // bit0 streaming, bit1 imu ok
  uint16_t odr_hz;
  uint16_t accel_fs_g;
  uint16_t gyro_fs_dps;
  float    accel_scale;    // g per LSB
  float    gyro_scale;     // dps per LSB
  uint32_t uptime_ms;
  uint16_t batt_mv;
  uint16_t overruns;       // FIFO overflowed: samples lost on the device
  uint16_t tx_drops;       // notification refused by the stack
  uint16_t mtu;            // negotiated ATT MTU; decides samples per packet
};

// Written to the control characteristic as [opcode, arg].
#define CMD_SET_ODR        0x01   // arg: 0=52 1=104 2=208 3=416 Hz
#define CMD_SET_ACCEL_FS   0x02   // arg: 0=2 1=4 2=8 3=16 g
#define CMD_SET_GYRO_FS    0x03   // arg: 0=245 1=500 2=1000 3=2000 dps
#define CMD_STREAM         0x10   // arg: 0=pause 1=resume
#define CMD_RESET_INDEX    0x11   // restart the sample index and flush the FIFO
#define CMD_IDENTIFY       0x20   // arg: seconds to blink (0 = 3)
