"""Simulierter Anfängerflug: ruckartig, schnell, stark geneigt, ungenauer Kompass.
Schreibt ein Pi-Log (.bin) mit Scans + ATTITUDE + DISTANCE_SENSOR wie pi_streamer.py --mav.
Aufruf: python sim_beginner.py OUT.bin [--yaw-max 300] [--seed 1]
"""
import math
import sys

import numpy as np

import protocol as P
from slam import level_points
from sim_tilt import raycast3d

SCAN_HZ, PTS, DT = 6.2, 600, 0.005
BOX = (1.2, 6.6, 2.4, 5.4)          # freier Flugbereich im Raum (x0, x1, y0, y1)


def make_flight(T, yaw_max_dps, rng):
    n = int(T / DT)
    t = np.arange(n) * DT
    # --- Gier: Ruhephasen + ruckartige Drehstöße ---
    rate = np.zeros(n)
    i = 0
    while i < n:
        i += int(rng.uniform(0.4, 1.8) / DT)                 # Pause
        ang = math.radians(rng.uniform(30, 200)) * rng.choice([-1, 1])
        peak = math.radians(rng.uniform(0.5, 1.0) * yaw_max_dps)
        dur = 2 * abs(ang) / peak                             # Dreiecksprofil
        k = int(dur / DT)
        if k < 2 or i + k >= n:
            continue
        prof = np.concatenate([np.linspace(0, 1, k // 2), np.linspace(1, 0, k - k // 2)])
        rate[i:i + k] += math.copysign(peak, ang) * prof
        i += k
    jitter = np.convolve(rng.normal(0, 1, n), np.ones(40) / 40, "same") * math.radians(60)
    yaw = np.cumsum(rate + jitter) * DT
    # --- Position: sprunghafte Wunschgeschwindigkeit, harte Beschleunigung ---
    pos = np.zeros((n, 2)); vel = np.zeros((n, 2)); acc = np.zeros((n, 2))
    p = np.array([(BOX[0] + BOX[1]) / 2, (BOX[2] + BOX[3]) / 2]); v = np.zeros(2)
    target_v = np.zeros(2); next_change = 0.0
    for j in range(n):
        if t[j] >= next_change:
            ang = rng.uniform(0, 2 * math.pi)
            target_v = rng.uniform(0.3, 1.5) * np.array([math.cos(ang), math.sin(ang)])
            next_change = t[j] + rng.uniform(0.8, 2.0)
        # an den Rändern zurücklenken
        for a_, lo, hi in ((0, BOX[0], BOX[1]), (1, BOX[2], BOX[3])):
            if p[a_] < lo + 0.4:
                target_v[a_] = 0.8
            if p[a_] > hi - 0.4:
                target_v[a_] = -0.8
        sp_ = np.hypot(*target_v)
        if sp_ > 1.5:
            target_v = target_v * 1.5 / sp_
        a = np.clip((target_v - v) * 4.0, -3.5, 3.5)          # bis 3,5 m/s²: ruckartig
        v = v + a * DT
        p = p + v * DT
        pos[j], vel[j], acc[j] = p, v, a
    # --- Neigung aus Beschleunigung (Nase runter beim Beschleunigen) + Wackeln ---
    c, s = np.cos(yaw), np.sin(yaw)
    a_fwd = c * acc[:, 0] + s * acc[:, 1]
    a_left = -s * acc[:, 0] + c * acc[:, 1]
    lim = math.radians(20)
    pitch = np.clip(-np.arctan(a_fwd / 9.81), -lim, lim) + math.radians(2) * np.sin(2 * math.pi * 1.7 * t)
    roll = np.clip(-np.arctan(a_left / 9.81), -lim, lim) + math.radians(2) * np.sin(2 * math.pi * 2.3 * t + 1)
    tilt = np.arccos(np.cos(roll) * np.cos(pitch))       # Gesamtneigung auf 20° begrenzen
    f = np.where(tilt > lim, lim / np.maximum(tilt, 1e-9), 1.0)
    roll, pitch = roll * f, pitch * f
    h = 1.2 + 0.25 * np.sin(2 * math.pi * t / 17)
    return t, pos, yaw, roll, pitch, h


def main():
    out = sys.argv[1]
    yaw_max = float(sys.argv[sys.argv.index("--yaw-max") + 1]) if "--yaw-max" in sys.argv else 300.0
    seed = int(sys.argv[sys.argv.index("--seed") + 1]) if "--seed" in sys.argv else 1
    jump = float(sys.argv[sys.argv.index("--jump") + 1]) if "--jump" in sys.argv else 8.0
    rng = np.random.default_rng(seed)
    T = 75.0
    t, pos, yaw, roll, pitch, h = make_flight(T, yaw_max, rng)
    at = lambda tt: min(int(tt / DT), len(t) - 1)
    msgs, gt = [], []
    # Kompass-/EKF-Gierfehler: Rauschen, Drift 3°/min, Sprung (Standard +8°) bei 40 s
    def fc_yaw_err(tt):
        return math.radians(0.3) * rng.normal() + math.radians(3 / 60) * tt + (math.radians(jump) if tt > 40 else 0.0)
    for tt in np.arange(0, T, 0.05):
        k = at(tt)
        msgs.append((tt, P.encode_pose(tt, roll[k] + math.radians(0.3) * rng.normal(),
                                       pitch[k] + math.radians(0.3) * rng.normal(),
                                       -(yaw[k] + fc_yaw_err(tt)), math.nan, math.nan, math.nan)))
        cr, sr, cp, sp = math.cos(roll[k]), math.sin(roll[k]), math.cos(pitch[k]), math.sin(pitch[k])
        down = np.array([[sp * cr, sr, -cp * cr]])
        r = raycast3d(pos[k, 0], pos[k, 1], h[k], down)[0] + rng.normal(0, 0.01)
        msgs.append((tt, P.encode_range(tt, r, 25)))
    dt = 1.0 / SCAN_HZ
    sid, ts = 0, 0.0
    a = np.arange(PTS) * 360.0 / PTS
    th = -np.radians(a)
    unit = np.column_stack((np.cos(th), np.sin(th)))
    while ts + dt < T:
        D = np.empty(PTS)
        for b in range(0, PTS, 10):                      # Pose alle 10 Punkte (~2,7 ms)
            k = at(ts + b / PTS * dt)
            lv, lz = level_points(unit[b:b + 10], roll[k], pitch[k])
            c, s = math.cos(yaw[k]), math.sin(yaw[k])
            wd = np.column_stack((c * lv[:, 0] - s * lv[:, 1], s * lv[:, 0] + c * lv[:, 1], lz))
            D[b:b + 10] = raycast3d(pos[k, 0], pos[k, 1], h[k], wd)
        D = D + rng.normal(0, 0.015, PTS)
        ok = np.isfinite(D) & (D < 12) & (D > 0.05) & (rng.random(PTS) > 0.05)
        te = ts + dt
        for m in P.encode_scan(sid, te, a[ok], np.round(D[ok] * 1000), np.full(ok.sum(), 120)):
            msgs.append((te, m))
        k = at(te)
        gt.append((pos[k, 0], pos[k, 1], yaw[k]))
        sid += 1
        ts += dt
    msgs.sort(key=lambda x: x[0])
    with open(out, "wb") as f:
        for _, m in msgs:
            f.write(P.LEN.pack(len(m)) + m)
    np.save(out + ".gt.npy", np.array(gt))
    rate = np.degrees(np.abs(np.diff(yaw))) / DT
    tilt = np.degrees(np.arccos(np.cos(roll) * np.cos(pitch)))
    spd = np.hypot(*np.diff(pos, axis=0).T) / DT
    print(f"{sid} Scans, {T:.0f} s | Drehrate max {rate.max():.0f}°/s (95% {np.percentile(rate, 95):.0f}°/s)"
          f" | Neigung max {tilt.max():.0f}° | Tempo max {spd.max():.1f} m/s")


if __name__ == "__main__":
    main()
