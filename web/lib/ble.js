// Web Bluetooth connection to the dog-vitals board.
//
// Emits decoded samples plus link statistics. The statistics matter as much as
// the data: at 208 Hz a dropped notification is invisible in a plot but ruins
// a rate estimate, so the sample index is checked for continuity on every
// packet and gaps are surfaced rather than smoothed over.

import {
  SERVICE_UUID, DATA_UUID, CTRL_UUID, STATUS_UUID,
  parsePacket, parseStatus, CMD,
} from './protocol.js';

/**
 * Everything that is true of the stream regardless of how it arrived.
 *
 * Both transports carry the identical bytes -- the bridge server forwards the
 * device's own notification payloads untouched -- so gap accounting, the
 * measured ODR and the delivered rate all belong here rather than being
 * implemented once per transport and drifting apart.
 */
export class LinkBase extends EventTarget {
  constructor() {
    super();
    this.status = null;
    this.reset();
  }

  reset() {
    this.packets = 0;
    this.samples = 0;
    this.gaps = 0;
    this.lost = 0;
    this.nextIdx = null;
    this.firstIdx = null;
    this.firstDevMs = null;
    this.lastIdx = null;
    this.lastDevMs = null;
    this.lastSeq = null;
    this.rateWindow = [];
  }

  /**
   * Measured sample rate from the device's own millisecond clock.
   *
   * The nominal ODR is not good enough to build a time base from: the
   * LSM6DS3's internal oscillator is trimmed to a few percent and drifts with
   * temperature, and a 2% error is a 2% error in every rate derived from it.
   * Host arrival times cannot be used instead -- delivery is bursty, so they
   * measure the transport, not the sensor.
   */
  get measuredOdr() {
    if (this.firstDevMs === null || this.lastDevMs === null) return null;
    const dMs = this.lastDevMs - this.firstDevMs;
    const dIdx = this.lastIdx - this.firstIdx;
    if (dMs < 2000 || dIdx <= 0) return null;   // too short to be meaningful
    return (dIdx * 1000) / dMs;
  }

  /** Samples per second as actually delivered to this page, over ~3 s. */
  get hostRate() {
    if (this.rateWindow.length < 2) return 0;
    const span = this.rateWindow.at(-1)[0] - this.rateWindow[0][0];
    if (span <= 0) return 0;
    const n = this.rateWindow.reduce((a, [, c]) => a + c, 0);
    return (n * 1000) / span;
  }

  ingestPacket(dv) {
    const p = parsePacket(dv);
    if (!p) return;

    // A jump in the index means the device produced samples that never
    // arrived -- either the link dropped them or the sensor FIFO overflowed.
    // Either way the recording has a hole at a known place, which is very
    // different from data that merely arrived late.
    if (this.nextIdx !== null && p.idx !== this.nextIdx) {
      this.gaps++;
      this.lost += Math.max(0, p.idx - this.nextIdx);
      this.emit('gap', { expected: this.nextIdx, got: p.idx });
    }
    this.nextIdx = p.idx + p.n;

    if (this.firstIdx === null) { this.firstIdx = p.idx; this.firstDevMs = p.tMs; }
    this.lastIdx = p.idx + p.n;
    this.lastDevMs = p.tMs;
    this.packets++;
    this.samples += p.n;
    this.lastSeq = p.seq;

    const now = performance.now();
    this.rateWindow.push([now, p.n]);
    while (this.rateWindow.length && now - this.rateWindow[0][0] > 3000) {
      this.rateWindow.shift();
    }
    this.emit('packet', p);
  }

  ingestStatus(dv) {
    const s = parseStatus(dv);
    if (s) { this.status = s; this.emit('status', s); }
    return s;
  }

  emit(type, detail) {
    this.dispatchEvent(new CustomEvent(type, { detail }));
  }
}

export class VitalsLink extends LinkBase {
  constructor() {
    super();
    this.device = null;
    this.server = null;
    this.dataChar = null;
    this.ctrlChar = null;
    this.statusChar = null;
  }

  get connected() {
    return !!(this.device && this.device.gatt && this.device.gatt.connected);
  }

  async connect() {
    this.device = await navigator.bluetooth.requestDevice({
      filters: [{ services: [SERVICE_UUID] }, { namePrefix: 'DogVitals' }],
      optionalServices: [SERVICE_UUID, 'battery_service', 'device_information'],
    });
    // requestDevice can hand back the same device object across reconnects,
    // so the listener is attached once rather than once per connect -- three
    // reconnects would otherwise fire three 'disconnected' events on the next
    // drop.
    if (!this.device.__dvBound) {
      this.device.__dvBound = true;
      this.device.addEventListener('gattserverdisconnected', () => {
        this.emit('disconnected');
      });
    }

    this.server = await this.device.gatt.connect();
    const svc = await this.server.getPrimaryService(SERVICE_UUID);
    this.dataChar   = await svc.getCharacteristic(DATA_UUID);
    this.ctrlChar   = await svc.getCharacteristic(CTRL_UUID);
    this.statusChar = await svc.getCharacteristic(STATUS_UUID);

    await this.refreshStatus();

    this.statusChar.addEventListener('characteristicvaluechanged', (e) => {
      this.ingestStatus(e.target.value);
    });
    await this.statusChar.startNotifications();

    this.dataChar.addEventListener('characteristicvaluechanged', (e) => {
      this.ingestPacket(e.target.value);
    });

    this.reset();
    await this.dataChar.startNotifications();
    this.emit('connected');
    return this.status;
  }

  async disconnect() {
    try { await this.dataChar?.stopNotifications(); } catch { /* going away anyway */ }
    if (this.device?.gatt?.connected) this.device.gatt.disconnect();
  }

  async refreshStatus() {
    return this.ingestStatus(await this.statusChar.readValue());
  }

  async send(op, arg = 0) {
    if (!this.ctrlChar) return;
    const b = Uint8Array.of(op, arg);
    // Write-without-response would be lost silently if the queue is full, and
    // these are one-shot configuration changes, not a stream.
    await this.ctrlChar.writeValueWithResponse(b);
  }

  setOdr(i)      { return this.send(CMD.SET_ODR, i); }
  setAccelFs(i)  { return this.send(CMD.SET_ACCEL_FS, i); }
  setGyroFs(i)   { return this.send(CMD.SET_GYRO_FS, i); }
  setStream(on)  { return this.send(CMD.STREAM, on ? 1 : 0); }
  resetIndex()   { return this.send(CMD.RESET_INDEX, 0); }
  identify(s= 3) { return this.send(CMD.IDENTIFY, s); }

}
