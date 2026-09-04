// Sample buffering, session recording and import/export.

const DB_NAME = 'dog-vitals';
const DB_VERSION = 1;
const STORE = 'sessions';

/**
 * A fixed-length window of uniformly-sampled data.
 *
 * The filters downstream all assume a constant sample interval, so the buffer
 * has to present one even though BLE does not. When the sample index jumps,
 * the missing samples are linearly interpolated and recorded in `fills` --
 * invented data, but invented in a way that is bounded, marked, and shown in
 * the UI rather than quietly blended in. Past `maxFillSec` the gap is too big
 * to bridge honestly and the window restarts instead; a rate computed across
 * two seconds of fabricated signal would be a number with nothing behind it.
 */
export class SampleBuffer {
  constructor(capacity = 208 * 300, maxFillSec = 1.5) {
    this.capacity = capacity;
    this.maxFillSec = maxFillSec;
    this.clear();
  }

  clear() {
    this.ax = new Float64Array(this.capacity);
    this.ay = new Float64Array(this.capacity);
    this.az = new Float64Array(this.capacity);
    this.gx = new Float64Array(this.capacity);
    this.gy = new Float64Array(this.capacity);
    this.gz = new Float64Array(this.capacity);
    this.len = 0;
    this.startIdx = null;    // device sample index of element 0
    this.nextIdx = null;
    this.fills = [];         // [start, end) positions that were interpolated
    this.filledSamples = 0;
  }

  #push(a, g) {
    if (this.len === this.capacity) {
      const keep = this.capacity - 1;
      const shift = this.capacity - keep;
      for (const arr of [this.ax, this.ay, this.az, this.gx, this.gy, this.gz]) {
        arr.copyWithin(0, shift);
      }
      this.len = keep;
      this.startIdx += shift;
      this.fills = this.fills
        .map(([s, e]) => [s - shift, e - shift])
        .filter(([, e]) => e > 0);
    }
    const i = this.len++;
    this.ax[i] = a[0]; this.ay[i] = a[1]; this.az[i] = a[2];
    this.gx[i] = g[0]; this.gy[i] = g[1]; this.gz[i] = g[2];
  }

  /**
   * @param packet parsed packet (raw int16 counts)
   * @param scales { accelScale, gyroScale } to convert to g and dps
   * @param fs     nominal sample rate, for the gap-size limit
   */
  add(packet, scales, fs) {
    const { accelScale: as, gyroScale: gs } = scales;

    if (this.startIdx === null) {
      this.startIdx = packet.idx;
      this.nextIdx = packet.idx;
    }

    if (packet.idx > this.nextIdx) {
      const missing = packet.idx - this.nextIdx;
      if (missing > this.maxFillSec * fs) {
        this.clear();
        this.startIdx = packet.idx;
        this.nextIdx = packet.idx;
      } else if (this.len > 0) {
        const p0 = this.len - 1;
        const a0 = [this.ax[p0], this.ay[p0], this.az[p0]];
        const g0 = [this.gx[p0], this.gy[p0], this.gz[p0]];
        const a1 = [packet.accel[0] * as, packet.accel[1] * as, packet.accel[2] * as];
        const g1 = [packet.gyro[0] * gs, packet.gyro[1] * gs, packet.gyro[2] * gs];
        const from = this.len;
        for (let k = 1; k <= missing; k++) {
          const f = k / (missing + 1);
          this.#push(
            [a0[0] + (a1[0] - a0[0]) * f, a0[1] + (a1[1] - a0[1]) * f, a0[2] + (a1[2] - a0[2]) * f],
            [g0[0] + (g1[0] - g0[0]) * f, g0[1] + (g1[1] - g0[1]) * f, g0[2] + (g1[2] - g0[2]) * f],
          );
        }
        this.fills.push([from, this.len]);
        this.filledSamples += missing;
      }
      this.nextIdx = packet.idx;
    }

    for (let i = 0; i < packet.n; i++) {
      this.#push(
        [packet.accel[i * 3] * as, packet.accel[i * 3 + 1] * as, packet.accel[i * 3 + 2] * as],
        [packet.gyro[i * 3] * gs, packet.gyro[i * 3 + 1] * gs, packet.gyro[i * 3 + 2] * gs],
      );
    }
    this.nextIdx = packet.idx + packet.n;
  }

  /** The most recent `seconds` of data, as plain arrays for the DSP layer. */
  window(seconds, fs) {
    const want = Math.min(this.len, Math.max(2, Math.round(seconds * fs)));
    const off = this.len - want;
    const slice = (arr) => arr.subarray(off, this.len);
    return {
      n: want,
      offset: off,
      ax: slice(this.ax), ay: slice(this.ay), az: slice(this.az),
      gx: slice(this.gx), gy: slice(this.gy), gz: slice(this.gz),
    };
  }
}

/** Accumulates a full session at native resolution, for saving and review. */
export class Recorder {
  constructor() { this.reset(); }

  reset() {
    this.recording = false;
    this.idx = [];
    this.accel = [];
    this.gyro = [];
    this.gaps = [];
    this.startedAt = null;
    this.meta = null;
  }

  start(meta) {
    this.reset();
    this.recording = true;
    this.startedAt = Date.now();
    this.meta = meta;
  }

  add(packet) {
    if (!this.recording) return;
    for (let i = 0; i < packet.n; i++) {
      this.idx.push(packet.idx + i);
      this.accel.push(packet.accel[i * 3], packet.accel[i * 3 + 1], packet.accel[i * 3 + 2]);
      this.gyro.push(packet.gyro[i * 3], packet.gyro[i * 3 + 1], packet.gyro[i * 3 + 2]);
    }
  }

  noteGap(g) { if (this.recording) this.gaps.push(g); }

  get sampleCount() { return this.idx.length; }

  stop(name) {
    this.recording = false;
    return {
      name: name || new Date(this.startedAt).toISOString().replace(/[:.]/g, '-'),
      startedAt: this.startedAt,
      endedAt: Date.now(),
      ...this.meta,
      // Raw counts are what the device produced; scales live in the metadata
      // so a session stays re-interpretable if a scale factor is ever
      // corrected, and so the file is a record rather than a derivation.
      idx: Uint32Array.from(this.idx),
      accel: Int16Array.from(this.accel),
      gyro: Int16Array.from(this.gyro),
      gaps: this.gaps,
    };
  }
}

// ------------------------------------------------------------------ IndexedDB

function openDb() {
  return new Promise((res, rej) => {
    const req = indexedDB.open(DB_NAME, DB_VERSION);
    req.onupgradeneeded = () => {
      const db = req.result;
      if (!db.objectStoreNames.contains(STORE)) {
        db.createObjectStore(STORE, { keyPath: 'id', autoIncrement: true });
      }
    };
    req.onsuccess = () => res(req.result);
    req.onerror = () => rej(req.error);
  });
}

export async function saveSession(session) {
  const db = await openDb();
  return new Promise((res, rej) => {
    const tx = db.transaction(STORE, 'readwrite');
    const req = tx.objectStore(STORE).add(session);
    req.onsuccess = () => res(req.result);
    req.onerror = () => rej(req.error);
  });
}

export async function listSessions() {
  const db = await openDb();
  return new Promise((res, rej) => {
    const out = [];
    const tx = db.transaction(STORE, 'readonly');
    tx.objectStore(STORE).openCursor().onsuccess = (e) => {
      const cur = e.target.result;
      if (!cur) { res(out.reverse()); return; }
      const v = cur.value;
      out.push({
        id: v.id, name: v.name, startedAt: v.startedAt, endedAt: v.endedAt,
        odrHz: v.odrHz, samples: v.idx.length, gaps: v.gaps?.length ?? 0,
      });
      cur.continue();
    };
    tx.onerror = () => rej(tx.error);
  });
}

export async function loadSession(id) {
  const db = await openDb();
  return new Promise((res, rej) => {
    const req = db.transaction(STORE, 'readonly').objectStore(STORE).get(id);
    req.onsuccess = () => res(req.result);
    req.onerror = () => rej(req.error);
  });
}

export async function deleteSession(id) {
  const db = await openDb();
  return new Promise((res, rej) => {
    const req = db.transaction(STORE, 'readwrite').objectStore(STORE).delete(id);
    req.onsuccess = () => res();
    req.onerror = () => rej(req.error);
  });
}

// --------------------------------------------------------------- import/export

export function sessionToCsv(s) {
  const lines = [
    `# dog-vitals session: ${s.name}`,
    `# started: ${new Date(s.startedAt).toISOString()}`,
    `# odr_hz: ${s.odrHz}   measured_odr_hz: ${s.measuredOdr ?? ''}`,
    `# accel_fs_g: ${s.accelFsG}   accel_g_per_lsb: ${s.accelScale}`,
    `# gyro_fs_dps: ${s.gyroFsDps}   gyro_dps_per_lsb: ${s.gyroScale}`,
    `# gaps: ${s.gaps?.length ?? 0}`,
    '# idx is the device sample index; jumps mark samples that were lost, not time that did not pass',
    'idx,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps',
  ];
  const as = s.accelScale, gs = s.gyroScale;
  for (let i = 0; i < s.idx.length; i++) {
    lines.push(
      `${s.idx[i]},` +
      `${(s.accel[i * 3] * as).toFixed(6)},${(s.accel[i * 3 + 1] * as).toFixed(6)},${(s.accel[i * 3 + 2] * as).toFixed(6)},` +
      `${(s.gyro[i * 3] * gs).toFixed(4)},${(s.gyro[i * 3 + 1] * gs).toFixed(4)},${(s.gyro[i * 3 + 2] * gs).toFixed(4)}`,
    );
  }
  return lines.join('\n');
}

export function sessionToJson(s) {
  return JSON.stringify({
    ...s,
    idx: Array.from(s.idx),
    accel: Array.from(s.accel),
    gyro: Array.from(s.gyro),
  });
}

/** Parses a CSV written by sessionToCsv, or the firmware's USB console output. */
export function csvToSession(text, name = 'imported') {
  const lines = text.split(/\r?\n/);
  let accelScale = 0.000061, gyroScale = 0.00875, odrHz = 208;
  let scaled = true;
  const idx = [], accel = [], gyro = [];

  for (const line of lines) {
    if (!line) continue;
    if (line.startsWith('#')) {
      let m = line.match(/accel_g_per_lsb[:= ]\s*([0-9.eE-]+)/);
      if (m) accelScale = parseFloat(m[1]);
      m = line.match(/gyro_dps_per_lsb[:= ]\s*([0-9.eE-]+)/);
      if (m) gyroScale = parseFloat(m[1]);
      m = line.match(/odr[_ ]?(?:hz)?[:= ]\s*(\d+)/);
      if (m) odrHz = parseInt(m[1], 10);
      // The firmware's console emits raw counts; a saved session emits g/dps.
      if (line.includes('raw int16')) scaled = false;
      continue;
    }
    if (/^idx/.test(line)) { scaled = !line.includes(',ax,'); continue; }
    const p = line.split(',');
    if (p.length < 7) continue;
    // The console format carries a t_ms column the saved format does not.
    const off = p.length >= 8 ? 2 : 1;
    const i = parseInt(p[0], 10);
    if (!Number.isFinite(i)) continue;
    const v = p.slice(off, off + 6).map(Number);
    if (v.some((x) => !Number.isFinite(x))) continue;
    idx.push(i);
    if (scaled) {
      accel.push(Math.round(v[0] / accelScale), Math.round(v[1] / accelScale), Math.round(v[2] / accelScale));
      gyro.push(Math.round(v[3] / gyroScale), Math.round(v[4] / gyroScale), Math.round(v[5] / gyroScale));
    } else {
      accel.push(v[0], v[1], v[2]);
      gyro.push(v[3], v[4], v[5]);
    }
  }

  return {
    name, startedAt: Date.now(), endedAt: Date.now(),
    odrHz, accelScale, gyroScale, accelFsG: 2, gyroFsDps: 245,
    idx: Uint32Array.from(idx),
    accel: Int16Array.from(accel),
    gyro: Int16Array.from(gyro),
    gaps: [],
  };
}

/** Expands a stored session back to uniform arrays, interpolating gaps. */
export function sessionToArrays(s) {
  const n = s.idx.length;
  if (!n) return { n: 0 };
  const total = s.idx[n - 1] - s.idx[0] + 1;
  const out = {
    ax: new Float64Array(total), ay: new Float64Array(total), az: new Float64Array(total),
    gx: new Float64Array(total), gy: new Float64Array(total), gz: new Float64Array(total),
  };
  const as = s.accelScale, gs = s.gyroScale;
  const base = s.idx[0];
  const filled = [];

  let prev = -1;
  for (let i = 0; i < n; i++) {
    const p = s.idx[i] - base;
    out.ax[p] = s.accel[i * 3] * as;
    out.ay[p] = s.accel[i * 3 + 1] * as;
    out.az[p] = s.accel[i * 3 + 2] * as;
    out.gx[p] = s.gyro[i * 3] * gs;
    out.gy[p] = s.gyro[i * 3 + 1] * gs;
    out.gz[p] = s.gyro[i * 3 + 2] * gs;
    if (prev >= 0 && p > prev + 1) {
      filled.push([prev + 1, p]);
      for (const k of ['ax', 'ay', 'az', 'gx', 'gy', 'gz']) {
        const a = out[k][prev], b = out[k][p];
        for (let q = prev + 1; q < p; q++) out[k][q] = a + ((b - a) * (q - prev)) / (p - prev);
      }
    }
    prev = p;
  }
  return { n: total, ...out, fills: filled };
}
