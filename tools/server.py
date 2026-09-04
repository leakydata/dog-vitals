#!/usr/bin/env python3
"""Serves the UI and bridges the board's BLE stream to it over a WebSocket.

Web Bluetooth is a fine transport when it works, but it is not available in
every browser, it is gated behind a permission prompt and a device picker every
single session, and on Linux it sits on top of the same BlueZ stack this
process can talk to directly with far more control. So the browser stops doing
Bluetooth at all: this server holds the connection, and the page becomes a
plain WebSocket client.

Frames pushed to the browser are the device's own bytes with a one-byte type
tag in front, so web/lib/protocol.js decodes them completely unchanged -- there
is exactly one definition of the wire format on the JS side, and it does not
care which transport delivered the bytes.
"""
import argparse
import asyncio
import json
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

from aiohttp import web, WSMsgType
from bleak import BleakClient, BleakScanner

BASE = "da7a{:04x}-1e3f-4b2c-9a5d-6f8e0c1b2a34"
UUID_DATA = BASE.format(0x0002)
UUID_CTRL = BASE.format(0x0003)
UUID_STAT = BASE.format(0x0004)

TAG_PACKET = 1
TAG_STATUS = 2

HDR = struct.Struct("<BBHII")
STATUS = struct.Struct("<BBHHHffIHHHH")

WEB_ROOT = Path(__file__).resolve().parent.parent / "web"
DATA_DIR = Path(__file__).resolve().parent.parent / "data"


class Bridge:
    def __init__(self):
        self.clients = {}      # ws -> outbound queue
        self.client = None
        self.connected = False
        self.device_addr = None
        self.status_raw = None
        self.scales = (0.000061, 0.00875)
        self.odr = 208
        self.packets = 0
        self.samples = 0
        self.gaps = 0
        self.lost = 0
        self._next_idx = None
        self.recorder = None
        self.record_path = None
        self.record_samples = 0
        self.last_error = None

    # ---------------------------------------------------------------- fan-out

    # One queue and one writer task per client.
    #
    # Broadcasting used to spawn a task per packet per client
    # (asyncio.create_task(self._send(...))), which lets several writes into
    # the same WebSocket run concurrently. aiohttp's writer is not safe that
    # way -- the frames interleave and the connection quietly stops behaving,
    # in a fashion that leaves the browser thinking its socket is still open
    # while nothing it sends is ever read. A single writer per client makes the
    # ordering explicit, and a bounded queue makes back-pressure a dropped
    # frame rather than an unbounded backlog.
    def broadcast(self, tag: int, payload: bytes):
        if not self.clients:
            return
        frame = bytes((tag,)) + payload
        for ws, q in list(self.clients.items()):
            if ws.closed:
                self.clients.pop(ws, None)
                continue
            try:
                q.put_nowait(frame)
            except asyncio.QueueFull:
                # A tab that has been backgrounded stops draining. Drop the
                # oldest and keep going: the sample index in the payload means
                # the client can see exactly what it missed.
                try:
                    q.get_nowait()
                    q.put_nowait(frame)
                except Exception:
                    pass

    async def writer(self, ws, q):
        """Drains one client's queue. The only thing that writes to that ws."""
        try:
            while True:
                frame = await q.get()
                if frame is None:
                    return
                if ws.closed:
                    return
                await ws.send_bytes(frame)
        except Exception:
            pass

    async def notify_state(self):
        msg = json.dumps({"type": "state", **self.state()})
        for ws in list(self.clients):
            if ws.closed:
                self.clients.pop(ws, None)
                continue
            try:
                await ws.send_str(msg)
            except Exception:
                self.clients.pop(ws, None)

    def state(self):
        return {
            "connected": self.connected,
            "address": self.device_addr,
            "packets": self.packets,
            "samples": self.samples,
            "gaps": self.gaps,
            "lost": self.lost,
            "recording": self.recorder is not None,
            "recordPath": str(self.record_path) if self.record_path else None,
            "recordSamples": self.record_samples,
            "error": self.last_error,
        }

    # ------------------------------------------------------------- recording

    def start_recording(self, name=None):
        # Close whatever was already open first. Without this, a second start
        # (a client that reconnected, or a caller whose stop never landed)
        # reopens the same path while the old handle still holds an offset --
        # and the file fills with NUL padding where the two disagree, which
        # parses as neither a number nor a comment.
        self.stop_recording()
        DATA_DIR.mkdir(exist_ok=True)
        stem = name or f"neck_{datetime.now():%Y%m%d_%H%M%S}"
        stem = "".join(c for c in stem if c.isalnum() or c in "-_") or "session"
        self.record_path = DATA_DIR / f"{stem}.csv"
        self.recorder = open(self.record_path, "w")
        a, g = self.scales
        self.recorder.write(f"# dog-vitals capture {datetime.now().isoformat()}\n")
        self.recorder.write(f"# odr_hz: {self.odr}\n")
        self.recorder.write(f"# accel_g_per_lsb: {a}\n")
        self.recorder.write(f"# gyro_dps_per_lsb: {g}\n")
        self.recorder.write("# idx,ax,ay,az,gx,gy,gz   (raw int16 counts)\n")
        self.recorder.write("idx,ax,ay,az,gx,gy,gz\n")
        self.record_samples = 0

    def stop_recording(self):
        if self.recorder:
            self.recorder.close()
            self.recorder = None

    # ------------------------------------------------------------- BLE loop

    def _on_data(self, _handle, data: bytearray):
        b = bytes(data)
        if len(b) < HDR.size:
            return
        _ver, n, _seq, idx, _t = HDR.unpack_from(b)

        if self._next_idx is not None and idx != self._next_idx:
            self.gaps += 1
            self.lost += max(0, idx - self._next_idx)
        self._next_idx = idx + n
        self.packets += 1
        self.samples += n

        self.broadcast(TAG_PACKET, b)

        if self.recorder:
            try:
                vals = struct.unpack_from(f"<{n*6}h", b, HDR.size)
                w = self.recorder.write
                for i in range(n):
                    gx, gy, gz, ax, ay, az = vals[i*6:i*6+6]
                    w(f"{idx+i},{ax},{ay},{az},{gx},{gy},{gz}\n")
                self.record_samples += n
            except Exception as e:
                self.last_error = f"record: {e}"

    def _on_status(self, _handle, data: bytearray):
        b = bytes(data)
        self.status_raw = b
        try:
            st = STATUS.unpack_from(b)
            self.odr = st[2]
            self.scales = (st[5], st[6])
        except Exception:
            pass
        self.broadcast(TAG_STATUS, b)

    async def run(self, address=None):
        """Connects and stays connected, retrying forever."""
        while True:
            try:
                dev = None
                for _ in range(3):
                    if address:
                        dev = await BleakScanner.find_device_by_address(address, timeout=12)
                    else:
                        dev = await BleakScanner.find_device_by_filter(
                            lambda d, ad: (d.name or "") == "DogVitals", timeout=12)
                    if dev:
                        break
                if not dev:
                    self.last_error = "DogVitals not found"
                    await self.notify_state()
                    await asyncio.sleep(3)
                    continue

                self.last_error = None
                # BlueZ refuses the first connect often enough that a single
                # attempt is not a reliable signal of anything.
                client = None
                for attempt in range(5):
                    try:
                        client = BleakClient(dev, timeout=25)
                        await client.connect()
                        break
                    except Exception as e:
                        self.last_error = f"connect {attempt+1}/5: {type(e).__name__}"
                        await self.notify_state()
                        client = None
                        await asyncio.sleep(2)
                if client is None:
                    await asyncio.sleep(3)
                    continue

                self.client = client
                self.connected = True
                self.device_addr = dev.address
                self.packets = self.samples = self.gaps = self.lost = 0
                self._next_idx = None
                self.last_error = None
                print(f"[bridge] connected to {dev.address}", flush=True)

                self._on_status(None, await client.read_gatt_char(UUID_STAT))
                await client.start_notify(UUID_STAT, self._on_status)
                await client.start_notify(UUID_DATA, self._on_data)
                await self.notify_state()

                last_state = time.monotonic()
                while client.is_connected:
                    await asyncio.sleep(0.5)
                    if time.monotonic() - last_state >= 2.0:
                        last_state = time.monotonic()
                        await self.notify_state()

                print("[bridge] link dropped", flush=True)
            except Exception as e:
                self.last_error = f"{type(e).__name__}: {e}"
                print(f"[bridge] {self.last_error}", flush=True)
            finally:
                self.connected = False
                self.client = None
                await self.notify_state()
                await asyncio.sleep(2)

    async def send_command(self, op: int, arg: int):
        if not (self.client and self.connected):
            return False
        try:
            await self.client.write_gatt_char(UUID_CTRL, bytes((op & 0xFF, arg & 0xFF)),
                                              response=True)
            return True
        except Exception as e:
            self.last_error = f"cmd: {e}"
            return False


# --------------------------------------------------------------------- routes

async def ws_handler(request):
    bridge = request.app["bridge"]
    ws = web.WebSocketResponse(heartbeat=20, max_msg_size=0)
    await ws.prepare(request)
    q = asyncio.Queue(maxsize=64)
    bridge.clients[ws] = q
    writer_task = asyncio.create_task(bridge.writer(ws, q))
    print(f"[bridge] browser connected ({len(bridge.clients)} total)", flush=True)

    await ws.send_str(json.dumps({"type": "state", **bridge.state()}))
    if bridge.status_raw:
        await ws.send_bytes(bytes((TAG_STATUS,)) + bridge.status_raw)

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                m = json.loads(msg.data)
            except Exception:
                continue
            if m.get("type") == "cmd":
                await bridge.send_command(int(m.get("op", 0)), int(m.get("arg", 0)))
            elif m.get("type") == "record":
                if m.get("on"):
                    bridge.start_recording(m.get("name"))
                    print(f"[bridge] recording -> {bridge.record_path}", flush=True)
                else:
                    n = bridge.record_samples
                    path = bridge.record_path
                    bridge.stop_recording()
                    print(f"[bridge] stopped: {path} ({n} samples)", flush=True)
                await bridge.notify_state()
    finally:
        bridge.clients.pop(ws, None)
        q.put_nowait(None)
        writer_task.cancel()
        print(f"[bridge] browser gone ({len(bridge.clients)} left)", flush=True)
    return ws


async def health(request):
    return web.json_response({"bridge": True, **request.app["bridge"].state()})


# The page and its modules must never be served from the browser cache.
#
# aiohttp sends no cache directives of its own, and Chrome will then apply
# heuristic freshness and reuse a file for hours without revalidating. That is
# invisible and it silently un-fixes bugs: a tab kept open across an edit ran
# an app.js old enough to predate server-side recording, so pressing Record
# started the browser's recorder, changed the button, and sent the server
# nothing at all. A minute of neck data went nowhere. This is a bench tool that
# gets edited while it is running -- there is no version of that where caching
# is worth the confusion.
@web.middleware
async def no_store(request, handler):
    resp = await handler(request)
    if not isinstance(resp, web.WebSocketResponse):
        resp.headers["Cache-Control"] = "no-store, must-revalidate"
        resp.headers["Pragma"] = "no-cache"
    return resp


async def on_start(app):
    app["ble_task"] = asyncio.create_task(app["bridge"].run(app["address"]))


async def on_cleanup(app):
    app["bridge"].stop_recording()
    app["ble_task"].cancel()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8420)
    ap.add_argument("--address", default=None, help="BLE address, if autodetect is ambiguous")
    a = ap.parse_args()

    app = web.Application(middlewares=[no_store])
    app["bridge"] = Bridge()
    app["address"] = a.address
    app.router.add_get("/api/health", health)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/lib/", WEB_ROOT / "lib")
    app.router.add_get("/", lambda r: web.FileResponse(WEB_ROOT / "index.html"))
    app.router.add_get("/app.js", lambda r: web.FileResponse(WEB_ROOT / "app.js"))
    app.on_startup.append(on_start)
    app.on_cleanup.append(on_cleanup)

    print(f"Dog Vitals:  http://localhost:{a.port}/   (BLE handled here, not in the browser)")
    web.run_app(app, host="127.0.0.1", port=a.port, print=None)


if __name__ == "__main__":
    sys.exit(main() or 0)
