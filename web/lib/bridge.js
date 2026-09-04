// WebSocket transport: the BLE link lives in tools/server.py, not in the page.
//
// Presents exactly the same surface as VitalsLink, so app.js does not know or
// care which transport it has. The server forwards the device's own
// notification payloads byte for byte with a one-byte tag in front, so the
// decoding in protocol.js is shared too -- there is one definition of the wire
// format, and swapping transports cannot introduce a second.

import { LinkBase } from './ble.js';
import { CMD } from './protocol.js';

const TAG_PACKET = 1;
const TAG_STATUS = 2;

export class BridgeLink extends LinkBase {
  constructor(url) {
    super();
    this.url = url || `ws://${location.host}/ws`;
    this.ws = null;
    this.serverState = null;
    this.wantOpen = false;
    this.connecting = null;
  }

  get connected() {
    // "Connected" means the board is streaming, not merely that the socket is
    // up. A live WebSocket to a server whose radio link has dropped is exactly
    // the state the UI must not report as healthy.
    return !!(this.ws && this.ws.readyState === WebSocket.OPEN
              && this.serverState?.connected);
  }

  get socketOpen() {
    return !!(this.ws && this.ws.readyState === WebSocket.OPEN);
  }

  connect() {
    this.wantOpen = true;
    if (this.connecting) return this.connecting;    // no overlapping attempts

    // Tear the previous socket down first.
    //
    // The reconnect path used to just create a new WebSocket and overwrite
    // this.ws, leaving the old one open with its handlers still bound. Several
    // sockets then accumulated: data kept arriving over an old one, so the UI
    // looked perfectly healthy, while this.ws -- the one every send goes
    // through -- pointed at a different socket the server was no longer
    // reading. Commands vanished with no error on either side.
    this.#teardown();

    this.connecting = new Promise((resolve, reject) => {
      let settled = false;
      const ws = new WebSocket(this.url);
      ws.binaryType = 'arraybuffer';
      this.ws = ws;

      ws.onopen = () => {
        this.connecting = null;
        this.reset();
        this.emit('connected');
        if (!settled) { settled = true; resolve(this.status); }
      };

      ws.onmessage = (e) => {
        if (this.ws !== ws) return;               // ignore a socket we replaced
        if (typeof e.data === 'string') {
          let m;
          try { m = JSON.parse(e.data); } catch { return; }
          if (m.type === 'state') {
            const was = this.serverState?.connected;
            this.serverState = m;
            if (was !== m.connected) this.emit(m.connected ? 'connected' : 'linkdown');
            this.emit('serverstate', m);
          }
          return;
        }
        const buf = new Uint8Array(e.data);
        if (buf.length < 2) return;
        const body = new DataView(e.data, 1);
        if (buf[0] === TAG_PACKET) this.ingestPacket(body);
        else if (buf[0] === TAG_STATUS) this.ingestStatus(body);
      };

      ws.onerror = () => {
        if (!settled) {
          settled = true;
          this.connecting = null;
          reject(new Error('cannot reach the bridge server'));
        }
      };

      ws.onclose = () => {
        if (this.ws !== ws) return;               // an old socket finishing up
        this.connecting = null;
        this.emit('disconnected');
        // The server survives the board going away and reconnects on its own,
        // so a closed socket means the server itself went down or was
        // restarted -- worth retrying, quietly, until it comes back.
        if (this.wantOpen) setTimeout(() => { if (this.wantOpen) this.connect(); }, 2000);
      };
    });
    return this.connecting;
  }

  #teardown() {
    const ws = this.ws;
    this.ws = null;
    if (!ws) return;
    ws.onopen = ws.onmessage = ws.onerror = ws.onclose = null;
    try { ws.close(); } catch { /* already gone */ }
  }

  async disconnect() {
    this.wantOpen = false;
    this.connecting = null;
    this.#teardown();
  }

  async refreshStatus() { return this.status; }

  #send(obj) {
    if (this.socketOpen) this.ws.send(JSON.stringify(obj));
  }

  async send(op, arg = 0) { this.#send({ type: 'cmd', op, arg }); }

  setOdr(i)      { return this.send(CMD.SET_ODR, i); }
  setAccelFs(i)  { return this.send(CMD.SET_ACCEL_FS, i); }
  setGyroFs(i)   { return this.send(CMD.SET_GYRO_FS, i); }
  setStream(on)  { return this.send(CMD.STREAM, on ? 1 : 0); }
  resetIndex()   { return this.send(CMD.RESET_INDEX, 0); }
  identify(s = 3) { return this.send(CMD.IDENTIFY, s); }

  /** Ask the server to also write a CSV next to the project's data/ folder. */
  serverRecord(on, name) { this.#send({ type: 'record', on: !!on, name }); }
}

/** True when this page is being served by the bridge server. */
export async function bridgeAvailable() {
  try {
    const r = await fetch('/api/health', { cache: 'no-store' });
    if (!r.ok) return false;
    const j = await r.json();
    return !!j.bridge;
  } catch {
    return false;
  }
}
