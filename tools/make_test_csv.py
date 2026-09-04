#!/usr/bin/env python3
"""Writes a synthetic session with a known breathing and heart rate.

Used to check the browser's analysis against an answer that is known in
advance. A capture from a board sitting on a desk proves the link works; it
proves nothing about whether the rate estimates are right, because there is no
rate in it.
"""
import argparse, math, random

ap = argparse.ArgumentParser()
ap.add_argument("--out", default="synthetic.csv")
ap.add_argument("--seconds", type=float, default=90)
ap.add_argument("--fs", type=float, default=208)
ap.add_argument("--brpm", type=float, default=15)     # breaths per minute
ap.add_argument("--bpm", type=float, default=72)      # heart beats per minute
a = ap.parse_args()

ACC = 0.000061
GYR = 0.00875
n = int(a.seconds * a.fs)
fb = a.brpm / 60.0
fh = a.bpm / 60.0

rows = []
for i in range(n):
    t = i / a.fs
    # Breathing: slow tilt of the gravity vector, tens of milli-g.
    breath = 0.030 * math.sin(2 * math.pi * fb * t)
    # Heartbeat: a short damped burst once per cycle, a few milli-g -- which is
    # what makes this hard, and what the band-pass has to dig out.
    ph = (t * fh) % 1.0
    beat = 0.0
    if ph < 0.12:
        beat = 0.004 * math.exp(-ph * 40) * math.sin(2 * math.pi * 14 * ph)
    ax = 0.02 + breath * 0.6 + beat + random.gauss(0, 0.0012)
    ay = -0.15 + breath + beat * 0.7 + random.gauss(0, 0.0012)
    az = 0.98 + breath * 0.3 + beat * 0.4 + random.gauss(0, 0.0012)
    gx = 30 * breath + 900 * beat + random.gauss(0, 0.05)
    gy = 12 * breath + 1400 * beat + random.gauss(0, 0.05)
    gz = -5 * breath + 600 * beat + random.gauss(0, 0.05)
    rows.append((i, ax, ay, az, gx, gy, gz))

with open(a.out, "w") as f:
    f.write(f"# synthetic: brpm={a.brpm} bpm={a.bpm}\n")
    f.write(f"# odr_hz: {int(a.fs)}\n")
    f.write(f"# accel_g_per_lsb: {ACC}\n")
    f.write(f"# gyro_dps_per_lsb: {GYR}\n")
    f.write("idx,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n")
    for i, ax, ay, az, gx, gy, gz in rows:
        f.write(f"{i},{ax:.6f},{ay:.6f},{az:.6f},{gx:.4f},{gy:.4f},{gz:.4f}\n")
print(f"wrote {a.out}: {n} samples, {a.brpm} brpm, {a.bpm} bpm")
