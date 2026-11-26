"""Simulator für die 3D-Karte (main.py --3d): Raum mit Fenster, Tür, Tisch, Regal; Scans auf 3 Höhen."""
import math
import pickle
import sys

import numpy as np

# (segment, z_unten, z_oben)
OBJ = [
    (((0, 0), (8, 0)), 0, 2.6), (((8, 0), (8, 6)), 0, 2.6), (((0, 6), (0, 0)), 0, 2.6),
    # Wand oben mit Tür (x 3..4, bis 2.0 m) und Fenster (x 5.5..7, 0.9..2.0 m)
    (((8, 6), (7, 6)), 0, 2.6), (((7, 6), (5.5, 6)), 0, 0.9), (((7, 6), (5.5, 6)), 2.0, 2.6),
    (((5.5, 6), (4, 6)), 0, 2.6), (((4, 6), (3, 6)), 2.0, 2.6), (((3, 6), (0, 6)), 0, 2.6),
    # Tisch 1.6 x 0.8 bis 0.75 m
    (((1, 1), (2.6, 1)), 0, 0.75), (((2.6, 1), (2.6, 1.8)), 0, 0.75),
    (((2.6, 1.8), (1, 1.8)), 0, 0.75), (((1, 1.8), (1, 1)), 0, 0.75),
    # Regal an der rechten Wand bis 1.8 m
    (((7.6, 2), (7.6, 4)), 0, 1.8), (((7.6, 4), (8, 4)), 0, 1.8), (((7.6, 2), (8, 2)), 0, 1.8),
    # Außenwand hinter Tür/Fenster (Flur)
    (((-1, 8), (9, 8)), 0, 2.6),
]


def raycast(x, y, phi, z):
    d = np.stack([np.cos(phi), np.sin(phi)], 1)
    best = np.full(len(phi), np.inf)
    for (p1, p2), z0, z1 in OBJ:
        if not (z0 <= z < z1):
            continue
        p1, p2 = np.array(p1, float), np.array(p2, float)
        e = p2 - p1
        den = d[:, 0] * e[1] - d[:, 1] * e[0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((p1[0] - x) * e[1] - (p1[1] - y) * e[0]) / den
            u = ((p1[0] - x) * d[:, 1] - (p1[1] - y) * d[:, 0]) / den
        ok = (np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(ok & (t < best), t, best)
    return best


def main():
    out = sys.argv[1]
    rng = np.random.default_rng(5)
    loop = [(1.0, 3.0), (5.0, 3.0), (6.5, 4.5), (4.0, 4.8), (2.0, 4.5), (1.0, 3.0)]
    data, n = [], 620
    t = 0.0
    for z in (0.5, 1.2, 1.9):
        for (x0, y0), (x1, y1) in zip(loop[:-1], loop[1:]):
            steps = int(math.hypot(x1 - x0, y1 - y0) / 0.05)
            yaw_new = math.atan2(y1 - y0, x1 - x0)
            if 'yaw' in dir():
                dy = (yaw_new - yaw + math.pi) % (2 * math.pi) - math.pi
                for j in range(1, int(abs(math.degrees(dy)) / 8) + 1):   # ~56 Grad/s auf der Stelle
                    yy = yaw + dy * j / (int(abs(math.degrees(dy)) / 8) + 1)
                    a = np.arange(n) * 360.0 / n
                    r = raycast(np.full(n, x0), np.full(n, y0), yy - np.radians(a), z) + rng.normal(0, 0.01, n)
                    ok = np.isfinite(r) & (r < 12)
                    data.append((t, a[ok], np.round(r[ok] * 1000), np.full(ok.sum(), 150, np.uint8), z))
                    t += 1 / 7
            yaw = yaw_new
            for i in range(steps):
                f = i / steps
                x, y = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
                a = np.arange(n) * 360.0 / n
                r = raycast(np.full(n, x), np.full(n, y), yaw - np.radians(a), z) + rng.normal(0, 0.01, n)
                ok = np.isfinite(r) & (r < 12)
                data.append((t, a[ok], np.round(r[ok] * 1000), np.full(ok.sum(), 150, np.uint8), z))
                t += 1 / 7
    with open(out, "wb") as f:
        pickle.dump(data, f)
    print(f"{len(data)} Scans auf Höhen 0.5 / 1.2 / 1.9 m")


if __name__ == "__main__":
    main()
