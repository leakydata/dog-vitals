#!/usr/bin/env python3
"""Captures a neck recording, triggered by picking the device up.

There is no way to synchronise "press start" with "hold the sensor against your
neck" when the two are done by the same person with one device between them, so
this does not try. It connects, watches for the board being lifted off the
desk, gives you time to position it, and then records. Everything from connect
onward is kept, with the transitions marked in the header, so the still-on-desk
stretch at the start is available as a noise floor to compare the neck data
against -- which is the only way to tell a real cardiac signal from a plausible
looking artefact.
"""
import argparse
import asyncio
import math
import struct
import sys
import time
from datetime import datetime

from bleak import BleakClient, BleakScanner

BASE = "da7a{:04x}-1e3f-4b2c-9a5d-6f8e0c1b2a34"
HDR = struct.Struct("<BBHII")
STATUS = struct.Struct("<BBHHHffIHHHH")

PICKUP_DPS = 25.0      # gyro magnitude that means "in a hand", not "on a desk"
SETTLE_SEC = 12.0      # time to get it positioned and hold still
BASELINE_KEEP = 20.0   # seconds of desk data to keep before the pickup


async def find(timeout=20):
    return await BleakScanner.find_device_by_filter(
        lambda d, ad: (d.name or "") == "DogVitals", timeout=timeout)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--wait", type=float, default=420.0, help="max wait for pickup")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    out = a.out or f"data/neck_{datetime.now():%Y%m%d_%H%M%S}.csv"

    dev = None
    for attempt in range(4):
        dev = await find()
        if dev:
            break
        print(f"scan attempt {attempt+1} found nothing, retrying", flush=True)
    if not dev:
        print("DogVitals not found -- is it powered?", file=sys.stderr)
        return 1
    print(f"found {dev.address}", flush=True)

    client = None
    for attempt in range(5):
        try:
            client = BleakClient(dev, timeout=25)
            await client.connect()
            break
        except Exception as e:
            print(f"connect attempt {attempt+1} failed ({type(e).__name__}), retrying", flush=True)
            client = None
            await asyncio.sleep(3)
    if client is None:
        print("could not connect", file=sys.stderr)
        return 1

    st = STATUS.unpack_from(bytes(await client.read_gatt_char(BASE.format(4))))
    odr, afs, gfs, ascale, gscale = st[2], st[3], st[4], st[5], st[6]
    print(f"connected: odr={odr} Hz  accel=+/-{afs}g  gyro=+/-{gfs}dps", flush=True)

    rows = []                 # (idx, ax,ay,az, gx,gy,gz) raw counts
    state = {"phase": "baseline", "pickup_idx": None, "rec_idx": None,
             "t_pickup": None, "done": False, "gaps": 0, "next": None}
    t0 = time.monotonic()

    def cb(_h, data):
        b = bytes(data)
        if len(b) < HDR.size:
            return
        ver, n, seq, idx, t_ms = HDR.unpack_from(b)
        if state["next"] is not None and idx != state["next"]:
            state["gaps"] += 1
        state["next"] = idx + n
        vals = struct.unpack_from(f"<{n*6}h", b, HDR.size)

        for i in range(n):
            gx, gy, gz, ax, ay, az = vals[i*6:i*6+6]
            rows.append((idx + i, ax, ay, az, gx, gy, gz))

        # Pickup detection on the gyro, which reacts to being lifted far more
        # decisively than the accelerometer does -- the accel barely changes if
        # the board is lifted without rotating.
        gm = max(math.hypot(vals[i*6]*gscale, vals[i*6+1]*gscale, vals[i*6+2]*gscale)
                 for i in range(n))

        now = time.monotonic()
        if state["phase"] == "baseline":
            # Ignore the first couple of seconds: connecting sometimes coincides
            # with the cable still swinging.
            if now - t0 > 2.0 and gm > PICKUP_DPS:
                state["phase"] = "settling"
                state["pickup_idx"] = idx
                state["t_pickup"] = now
                print(f"[{now-t0:5.1f}s] picked up ({gm:.0f} dps) -- "
                      f"position it and hold still, recording starts in {SETTLE_SEC:.0f}s",
                      flush=True)
            elif len(rows) > BASELINE_KEEP * odr * 1.5:
                del rows[:int(len(rows) - BASELINE_KEEP * odr)]
        elif state["phase"] == "settling":
            if now - state["t_pickup"] >= SETTLE_SEC:
                state["phase"] = "recording"
                state["rec_idx"] = idx
                print(f"[{now-t0:5.1f}s] RECORDING {a.seconds:.0f}s -- hold still, "
                      f"breathe normally", flush=True)
        elif state["phase"] == "recording":
            if (idx - state["rec_idx"]) / odr >= a.seconds:
                state["done"] = True

    await client.start_notify(BASE.format(2), cb)
    print(f"streaming. Pick the board up and hold it against your neck "
          f"(waiting up to {a.wait/60:.0f} min)", flush=True)

    deadline = time.monotonic() + a.wait
    while not state["done"] and time.monotonic() < deadline:
        await asyncio.sleep(0.25)
        if not client.is_connected:
            print("link dropped", file=sys.stderr)
            break

    try:
        await client.stop_notify(BASE.format(2))
        await client.disconnect()
    except Exception:
        pass

    if state["pickup_idx"] is None:
        print("never detected a pickup -- nothing useful captured", file=sys.stderr)
        return 2

    base = rows[0][0]
    with open(out, "w") as f:
        f.write(f"# dog-vitals neck capture {datetime.now().isoformat()}\n")
        f.write(f"# odr_hz: {odr}\n")
        f.write(f"# accel_g_per_lsb: {ascale}\n")
        f.write(f"# gyro_dps_per_lsb: {gscale}\n")
        f.write(f"# accel_fs_g: {afs}   gyro_fs_dps: {gfs}\n")
        f.write(f"# first_idx: {base}\n")
        f.write(f"# pickup_idx: {state['pickup_idx']}\n")
        f.write(f"# record_idx: {state['rec_idx']}\n")
        f.write(f"# gaps: {state['gaps']}\n")
        f.write("# idx,ax,ay,az,gx,gy,gz   (raw int16 counts)\n")
        f.write("idx,ax,ay,az,gx,gy,gz\n")
        for r in rows:
            f.write(",".join(str(v) for v in r) + "\n")

    dur = (rows[-1][0] - base) / odr
    rec = ((rows[-1][0] - state["rec_idx"]) / odr) if state["rec_idx"] else 0
    print(f"\nwrote {out}")
    print(f"  {len(rows)} samples, {dur:.1f}s total, {rec:.1f}s of neck recording, "
          f"{state['gaps']} gaps")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
