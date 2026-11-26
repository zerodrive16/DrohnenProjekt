"""Simulierte Drohne: kippt, steigt/sinkt, dreht sich. Schreibt ein Pi-Log (.bin) mit
Scans + ATTITUDE + DISTANCE_SENSOR, wie pi_streamer.py --mav es aufzeichnen würde.
Aufruf: python sim_tilt.py OUT.bin [--tilt GRAD]
"""
import math
import sys

import numpy as np

import protocol as P
from slam import level_points
from sim3d import OBJ

CEIL = 2.6
SCAN_HZ, PTS = 6.2, 600


def raycast3d(x, y, h, dirs):
    """dirs: (N,3) Weltrichtungen (x, y, z oben). -> Distanz entlang des Strahls."""
    hn = np.hypot(dirs[:, 0], dirs[:, 1])
    dh = dirs[:, :2] / np.maximum(hn[:, None], 1e-9)
    best = np.full(len(dirs), np.inf)
    for (p1, p2), z0, z1 in OBJ:
        p1, p2 = np.array(p1, float), np.array(p2, float)
        e = p2 - p1
        den = dh[:, 0] * e[1] - dh[:, 1] * e[0]
        with np.errstate(divide="ignore", invalid="ignore"):
            th = ((p1[0] - x) * e[1] - (p1[1] - y) * e[0]) / den
            u = ((p1[0] - x) * dh[:, 1] - (p1[1] - y) * dh[:, 0]) / den
            r = th / hn
        z = h + dirs[:, 2] * r
        ok = (np.abs(den) > 1e-9) & (th > 0) & (u >= 0) & (u <= 1) & (z >= z0) & (z < z1)
        best = np.where(ok & (r < best), r, best)
    with np.errstate(divide="ignore"):
        floor = np.where(dirs[:, 2] < -1e-6, h / -dirs[:, 2], np.inf)
        ceil = np.where(dirs[:, 2] > 1e-6, (CEIL - h) / dirs[:, 2], np.inf)
    return np.minimum(best, np.minimum(floor, ceil))


def make_traj(tilt_deg):
    loop = [(1.0, 3.0), (5.0, 3.0), (6.5, 4.5), (4.0, 4.8), (2.0, 4.5), (1.0, 3.0)]
    pts = loop * 3 + [loop[0]]
    seg_len = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:])]
    v = 0.4
    T = sum(seg_len) / v
    cum = np.cumsum([0] + seg_len)
    A = math.radians(tilt_deg)

    def pose(t):
        s = min(t * v, cum[-1] - 1e-6)
        i = np.searchsorted(cum, s, side="right") - 1
        f = (s - cum[i]) / seg_len[i]
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        x, y = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
        # Gierwinkel: Richtung des Segments, an Ecken weich überblendet
        def yaw_of(k):
            (a, b), (c, d) = pts[k], pts[k + 1]
            return math.atan2(d - b, c - a)
        yaw = yaw_of(i)
        if f > 0.8 and i + 1 < len(pts) - 1:
            dy = (yaw_of(i + 1) - yaw + math.pi) % (2 * math.pi) - math.pi
            yaw += dy * (f - 0.8) / 0.4
        elif f < 0.2 and i > 0:
            dy = (yaw - yaw_of(i - 1) + math.pi) % (2 * math.pi) - math.pi
            yaw -= dy * (0.2 - f) / 0.4
        h = 1.15 - 0.75 * math.cos(2 * math.pi * t / T * 1.5)        # 0,4 ... 1,9 m
        roll = A * math.sin(2 * math.pi * 0.37 * t) + 0.3 * A * math.sin(2 * math.pi * 1.9 * t)
        pitch = A * math.sin(2 * math.pi * 0.23 * t + 1.0) + 0.3 * A * math.sin(2 * math.pi * 2.3 * t)
        return x, y, yaw, h, roll, pitch

    return pose, T


def main():
    out = sys.argv[1]
    tilt = float(sys.argv[sys.argv.index("--tilt") + 1]) if "--tilt" in sys.argv else 7.0
    rng = np.random.default_rng(7)
    pose, T = make_traj(tilt)
    msgs, gt = [], []
    # Lage + Höhenmesser mit 20 Hz
    for t in np.arange(0, T, 0.05):
        x, y, yaw, h, roll, pitch = pose(t)
        msgs.append((t, P.encode_pose(t, roll, pitch, -yaw, math.nan, math.nan, math.nan)))
        # Höhenmesser zeigt fahrzeugfest nach unten
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        down = np.array([[sp * cr, sr, -cp * cr]])    # Richtung in Welt (x vorne-level, y links, z oben) ~ für Boden egal
        r = raycast3d(x, y, h, down)[0] + rng.normal(0, 0.01)
        msgs.append((t, P.encode_range(t, r, 25)))
    # Scans
    dt = 1.0 / SCAN_HZ
    sid = 0
    t = 0.0
    while t + dt < T:
        ts = t + np.arange(PTS) / PTS * dt
        a = np.arange(PTS) * 360.0 / PTS
        th = -np.radians(a)                                # LiDAR-Winkel -> mathematisch
        unit = np.column_stack((np.cos(th), np.sin(th)))
        D = np.empty(PTS)
        for k in range(0, PTS, 50):                        # Pose blockweise (je ~13 ms)
            x, y, yaw, h, roll, pitch = pose(ts[k])
            lv, lz = level_points(unit[k:k + 50], roll, pitch)
            c, s = math.cos(yaw), math.sin(yaw)
            wd = np.column_stack((c * lv[:, 0] - s * lv[:, 1], s * lv[:, 0] + c * lv[:, 1], lz))
            D[k:k + 50] = raycast3d(x, y, h, wd)
        D = D + rng.normal(0, 0.015, PTS)
        ok = np.isfinite(D) & (D < 12) & (D > 0.05) & (rng.random(PTS) > 0.05)
        te = t + dt
        for m in P.encode_scan(sid, te, a[ok], np.round(D[ok] * 1000), np.full(ok.sum(), 120)):
            msgs.append((te, m))
        gt.append(pose(te)[:3])
        sid += 1
        t += dt
    msgs.sort(key=lambda x: x[0])
    with open(out, "wb") as f:
        for _, m in msgs:
            f.write(P.LEN.pack(len(m)) + m)
    np.save(out + ".gt.npy", np.array(gt))
    print(f"{sid} Scans, {T:.0f} s, Neigung bis ±{tilt * 1.3:.0f}°, Höhe 0,4-1,9 m")


if __name__ == "__main__":
    main()
