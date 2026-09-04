// Wire format shared with the firmware. Mirrors firmware/dog_vitals_imu/protocol.h
// -- the two describe the same bytes, and the only compatibility check is the
// version byte.

const UUID = (id) =>
  `da7a${id.toString(16).padStart(4, '0')}-1e3f-4b2c-9a5d-6f8e0c1b2a34`;

export const SERVICE_UUID = UUID(0x0001);
export const DATA_UUID    = UUID(0x0002);
export const CTRL_UUID    = UUID(0x0003);
export const STATUS_UUID  = UUID(0x0004);

export const PROTO_VERSION = 1;

export const HEADER_BYTES = 12;
export const SAMPLE_BYTES = 12;   // six int16: gx gy gz ax ay az

export const CMD = {
  SET_ODR:      0x01,   // 0=52 1=104 2=208 3=416 Hz
  SET_ACCEL_FS: 0x02,   // 0=2 1=4 2=8 3=16 g
  SET_GYRO_FS:  0x03,   // 0=245 1=500 2=1000 3=2000 dps
  STREAM:       0x10,
  RESET_INDEX:  0x11,
  IDENTIFY:     0x20,
};

export const ODR_CHOICES   = [52, 104, 208, 416];
export const ACCEL_CHOICES = [2, 4, 8, 16];
export const GYRO_CHOICES  = [245, 500, 1000, 2000];

/**
 * Decodes one data notification.
 *
 * Returns the samples as raw int16 counts plus the header. Scaling is left to
 * the caller because the scale factors come from the status characteristic and
 * can change mid-session if the range is reconfigured -- baking last-known
 * scales in here would silently mis-scale the packets either side of a change.
 */
export function parsePacket(dv) {
  if (dv.byteLength < HEADER_BYTES) return null;
  const ver = dv.getUint8(0);
  const n   = dv.getUint8(1);
  const seq = dv.getUint16(2, true);
  const idx = dv.getUint32(4, true);
  const tMs = dv.getUint32(8, true);
  if (dv.byteLength < HEADER_BYTES + n * SAMPLE_BYTES) return null;

  const gyro  = new Int16Array(n * 3);
  const accel = new Int16Array(n * 3);
  let o = HEADER_BYTES;
  for (let i = 0; i < n; i++) {
    gyro[i * 3 + 0]  = dv.getInt16(o + 0,  true);
    gyro[i * 3 + 1]  = dv.getInt16(o + 2,  true);
    gyro[i * 3 + 2]  = dv.getInt16(o + 4,  true);
    accel[i * 3 + 0] = dv.getInt16(o + 6,  true);
    accel[i * 3 + 1] = dv.getInt16(o + 8,  true);
    accel[i * 3 + 2] = dv.getInt16(o + 10, true);
    o += SAMPLE_BYTES;
  }
  return { ver, n, seq, idx, tMs, gyro, accel };
}

/** Decodes the 28-byte status block. */
export function parseStatus(dv) {
  if (dv.byteLength < 28) return null;
  const flags = dv.getUint8(1);
  return {
    ver:         dv.getUint8(0),
    streaming:   (flags & 0x01) !== 0,
    imuOk:       (flags & 0x02) !== 0,
    odrHz:       dv.getUint16(2, true),
    accelFsG:    dv.getUint16(4, true),
    gyroFsDps:   dv.getUint16(6, true),
    accelScale:  dv.getFloat32(8, true),    // g per LSB
    gyroScale:   dv.getFloat32(12, true),   // dps per LSB
    uptimeMs:    dv.getUint32(16, true),
    battMv:      dv.getUint16(20, true),
    overruns:    dv.getUint16(22, true),
    txDrops:     dv.getUint16(24, true),
    mtu:         dv.getUint16(26, true),
  };
}

export function batteryPercent(mv) {
  if (!mv) return null;
  return Math.max(0, Math.min(100, Math.round((mv - 3300) / (4150 - 3300) * 100)));
}
