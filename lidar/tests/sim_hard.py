"""Harter Simulator: Handbewegung mit schnellen Drehungen, Verzerrung innerhalb des Scans.

Jeder Messpunkt wird zu seinem eigenen Zeitpunkt aufgenommen (wie beim echten,
rotierenden LiDAR). Ground Truth = Pose am ENDE jedes Scans.
Aufruf: python sim_hard.py OUT.pkl [--spin GRAD_PRO_S]
"""
import math
import pickle
import sys

import numpy as np

from sim_room import raycast

SCAN_HZ = 7.0
PTS = 620


def waypoints_to_traj(spin):
    """Gibt Funktion t -> (x, y, yaw) und Gesamtdauer zurück."""
    segs = []  # (dauer, start_pose, end_pose)
    p = np.array([1.5, 1.0, 0.0])

    def go(x, y, v=0.45):
        nonlocal p
        yaw = math.atan2(y - p[1], x - p[0])
        # erst drehen (mit spin Grad/s), dann gehen
        dyaw = (yaw - p[2] + math.pi) % (2 * math.pi) - math.pi
        if abs(dyaw) > 1e-3:
            q = p.copy(); q[2] = p[2] + dyaw
            segs.append((abs(math.degrees(dyaw)) / spin, p.copy(), q)); p = q
        q = np.array([x, y, p[2]])
        segs.append((math.hypot(x - p[0], y - p[1]) / v, p.copy(), q)); p = q

    def turn(deg):
        nonlocal p
        q = p.copy(); q[2] += math.radians(deg)
        segs.append((abs(deg) / spin, p.copy(), q)); p = q

    go(5, 1.0); turn(360); go(9, 2.5); go(5, 2.8); turn(-200); go(4.5, 6.5)
    go(1.2, 6.5); turn(270); go(1.2, 4); go(1.5, 1.2)
    times = np.cumsum([0] + [s[0] for s in segs])

    def pose(t):
        i = min(np.searchsorted(times, t, side="right") - 1, len(segs) - 1)
        d, a, b = segs[i]
        f = 0.0 if d == 0 else min(max((t - times[i]) / d, 0), 1)
        f = 0.5 - 0.5 * math.cos(math.pi * f)          # sanft beschleunigen/bremsen
        x = a + f * (b - a)
        # Handwackeln
        x[2] += 0.03 * math.sin(2 * math.pi * 1.3 * t) + 0.015 * math.sin(2 * math.pi * 3.1 * t)
        x[0] += 0.02 * math.sin(2 * math.pi * 0.9 * t)
        return x

    return pose, times[-1]


def main():
    out = sys.argv[1]
    spin = float(sys.argv[sys.argv.index("--spin") + 1]) if "--spin" in sys.argv else 90.0
    rng = np.random.default_rng(3)
    pose, T = waypoints_to_traj(spin)
    data, gt = [], []
    t = 0.0
    dt = 1.0 / SCAN_HZ
    while t + dt < T:
        ts = t + np.arange(PTS) / PTS * dt                # Aufnahmezeit je Punkt
        P = np.array([pose(x) for x in ts])
        a = np.arange(PTS) * 360.0 / PTS                  # Winkel in Aufnahmereihenfolge
        phi = P[:, 2] - np.radians(a)
        r = raycast(P[:, 0], P[:, 1], phi)        # jeder Strahl mit eigenem Ursprung
        r = r + rng.normal(0, 0.015, PTS)
        ok = np.isfinite(r) & (r < 12) & (rng.random(PTS) > 0.05)
        data.append((t + dt, a[ok], np.round(r[ok] * 1000), np.full(ok.sum(), 150, np.uint8)))
        gt.append(pose(t + dt))
        t += dt
    with open(out, "wb") as f:
        pickle.dump(data, f)
    np.save(out + ".gt.npy", np.array(gt))
    print(f"{len(data)} Scans, {T:.0f} s, Drehrate {spin:.0f} Grad/s")


if __name__ == "__main__":
    main()
