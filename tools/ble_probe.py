#!/usr/bin/env python3
"""Headless client for the dog-vitals board.

Connects, prints the status block, subscribes to the IMU stream and reports
what actually arrives: sample rate measured against the host clock, the ODR
implied by the device's own timestamps, and any gap in the sample index.

The sample index is what makes the gap check meaningful. BLE will happily
deliver a packet tens of milliseconds late, so wall-clock arrival times say
nothing about whether data was lost -- but the index is generated per sample on
the device, so a jump in it is a real hole and a smooth run is real continuity.

  ./ble_probe.py                 # 10 s summary
  ./ble_probe.py --seconds 60 --csv run.csv
"""
import argparse
import asyncio
import struct
import sys
import time

from bleak import BleakClient, BleakScanner

BASE = "da7a{:04x}-1e3f-4b2c-9a5d-6f8e0c1b2a34"
UUID_SVC = BASE.format(0x0001)
UUID_DATA = BASE.format(0x0002)
UUID_CTRL = BASE.format(0x0003)
UUID_STAT = BASE.format(0x0004)

HDR = struct.Struct("<BBHII")          # ver, n, seq, idx, t_ms
STATUS = struct.Struct("<BBHHHffIHHHH")


def decode_status(b: bytes) -> dict:
    if len(b) < STATUS.size:
        return {"error": f"short status: {len(b)} bytes, want {STATUS.size}"}
    (ver, flags, odr, afs, gfs, ascale, gscale,
     uptime, batt, over, drops, mtu) = STATUS.unpack_from(b)
    return {
        "ver": ver, "streaming": bool(flags & 1), "imu_ok": bool(flags & 2),
        "odr_hz": odr, "accel_fs_g": afs, "gyro_fs_dps": gfs,
        "accel_scale": ascale, "gyro_scale": gscale,
        "uptime_s": uptime / 1000.0, "batt_mv": batt,
        "fifo_overruns": over, "tx_drops": drops, "mtu": mtu,
    }


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--address", default=None)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    print("scanning for DogVitals ...")
    dev = None
    if args.address:
        dev = await BleakScanner.find_device_by_address(args.address, timeout=15)
    else:
        dev = await BleakScanner.find_device_by_filter(
            lambda d, ad: (d.name or "") == "DogVitals"
            or UUID_SVC in [u.lower() for u in ad.service_uuids],
            timeout=15,
        )
    if dev is None:
        print("not found -- is it powered and advertising?", file=sys.stderr)
        return 1
    print(f"found {dev.address}")

    state = {
        "packets": 0, "samples": 0, "gaps": 0, "lost": 0,
        "next_idx": None, "first": None, "last": None,
        "first_t": None, "last_t": None, "accel": None, "gyro": None,
    }
    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("idx,t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n")

    scales = {"a": 0.000061, "g": 0.00875}

    def on_data(_h, data: bytearray):
        if len(data) < HDR.size:
            return
        ver, n, seq, idx, t_ms = HDR.unpack_from(data)
        body = data[HDR.size:]
        if len(body) < n * 12:
            return

        # A jump means the device produced samples this client never saw.
        if state["next_idx"] is not None and idx != state["next_idx"]:
            state["gaps"] += 1
            state["lost"] += max(0, idx - state["next_idx"])
        state["next_idx"] = idx + n

        state["packets"] += 1
        state["samples"] += n
        now = time.monotonic()
        if state["first"] is None:
            state["first"], state["first_t"] = idx, (now, t_ms)
        state["last"], state["last_t"] = idx + n, (now, t_ms)

        vals = struct.unpack_from(f"<{n * 6}h", body)
        state["gyro"] = vals[0:3]
        state["accel"] = vals[3:6]
        if csv:
            a, g = scales["a"], scales["g"]
            for k in range(n):
                gx, gy, gz, ax, ay, az = vals[k * 6:k * 6 + 6]
                csv.write(f"{idx + k},{t_ms},{ax*a:.6f},{ay*a:.6f},{az*a:.6f},"
                          f"{gx*g:.4f},{gy*g:.4f},{gz*g:.4f}\n")

    async with BleakClient(dev, timeout=20) as client:
        print(f"connected (mtu {client.mtu_size})")

        raw = await client.read_gatt_char(UUID_STAT)
        st = decode_status(bytes(raw))
        print("status:")
        for k, v in st.items():
            print(f"  {k:16} {v}")
        if "accel_scale" in st and st["accel_scale"] > 0:
            scales["a"], scales["g"] = st["accel_scale"], st["gyro_scale"]

        if not st.get("imu_ok"):
            print("\n!! the board reports the IMU did not initialise", file=sys.stderr)

        await client.start_notify(UUID_DATA, on_data)
        print(f"\nstreaming for {args.seconds:g}s ...")
        await asyncio.sleep(args.seconds)
        await client.stop_notify(UUID_DATA)

        print(f"\npackets      {state['packets']}")
        print(f"samples      {state['samples']}")
        if state["packets"] and state["first_t"]:
            wall = state["last_t"][0] - state["first_t"][0]
            devms = state["last_t"][1] - state["first_t"][1]
            span = state["last"] - state["first"]
            if wall > 0:
                print(f"host rate    {state['samples'] / wall:.1f} Hz")
            if devms > 0:
                # The sensor's own clock, which is what the time base must use.
                print(f"device ODR   {span * 1000.0 / devms:.2f} Hz  "
                      f"(nominal {st.get('odr_hz')})")
        print(f"index gaps   {state['gaps']} ({state['lost']} samples lost)")
        if state["accel"]:
            a, g = scales["a"], scales["g"]
            print("last accel   " + "  ".join(f"{v*a:+.4f}g" for v in state["accel"]))
            print("last gyro    " + "  ".join(f"{v*g:+.2f}dps" for v in state["gyro"]))

        raw = await client.read_gatt_char(UUID_STAT)
        st2 = decode_status(bytes(raw))
        print(f"overruns     {st2.get('fifo_overruns')}   tx_drops {st2.get('tx_drops')}")
        print(f"battery      {st2.get('batt_mv')} mV")

    if csv:
        csv.close()
        print(f"\nwrote {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
