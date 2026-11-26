"""Simulierter L-förmiger Raum (2D) + Erzeugung echter LiDAR-Bytepakete."""
import math
import pickle
import struct
import sys

import numpy as np

from sources import TMiniPlus

# Raum: L-förmig, 10 x 8 m, mit Kisten und Türnische
WALLS = [((0, 0), (10, 0)), ((10, 0), (10, 4)), ((10, 4), (6, 4)), ((6, 4), (6, 8)),
         ((6, 8), (0, 8)), ((0, 8), (0, 0)),
         ((2, 2), (3, 2)), ((3, 2), (3, 3)), ((3, 3), (2, 3)), ((2, 3), (2, 2)),
         ((8, 1), (8.6, 1)), ((8.6, 1), (8.6, 1.4)), ((8.6, 1.4), (8, 1.4)), ((8, 1.4), (8, 1)),
         ((4, 6), (4, 7.2)), ((1, 5), (2.5, 5))]
W = np.array(WALLS, dtype=float)


def raycast(x, y, phi):
    d = np.stack([np.cos(phi), np.sin(phi)], 1)
    p1, p2 = W[:, 0], W[:, 1]
    e = p2 - p1
    best = np.full(len(phi), np.inf)
    for k in range(len(W)):
        den = d[:, 0] * e[k, 1] - d[:, 1] * e[k, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((p1[k, 0] - x) * e[k, 1] - (p1[k, 1] - y) * e[k, 0]) / den
            u = ((p1[k, 0] - x) * d[:, 1] - (p1[k, 1] - y) * d[:, 0]) / den
        ok = (np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(ok & (t < best), t, best)
    return best


def sim_scan(pose, rng, n=620):
    a = (np.arange(n) * 360.0 / n + rng.uniform(0, 0.5)) % 360   # gleichmäßig, Uhrzeigersinn
    phi = pose[2] - np.radians(a)
    r = raycast(pose[0], pose[1], phi) + rng.normal(0, 0.01, n)
    ok = np.isfinite(r) & (r < 12)
    return a[ok], np.round(r[ok] * 1000)


def trajectory():
    pts = [(1.5, 1.0), (5, 1.0), (9, 2.5), (5, 2.8), (4.5, 6.5), (1.2, 6.5), (1.2, 4)]
    out = []
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        steps = int(math.hypot(x1 - x0, y1 - y0) / 0.06)   # ~0.4 m/s bei 7 Hz
        for i in range(steps):
            f = i / steps
            out.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0), math.atan2(y1 - y0, x1 - x0)))
    # Gierwinkel glätten (keine Sprünge an Ecken), plus Wackeln
    tr = np.array(out)
    yaw = np.unwrap(tr[:, 2])
    k = 15
    yaw = np.convolve(np.pad(yaw, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), "valid")
    tr[:, 2] = yaw + 0.05 * np.sin(np.arange(len(tr)) * 0.3)
    return tr


def build_packets(a, d):
    """Scan im YDLIDAR-Format mit 3 Byte/Punkt (inkl. Start-Paket)."""
    out = bytearray()

    def pkt(ct, angs, dists):
        lsn = len(angs)
        fsa = (int(round(angs[0] * 64)) << 1) | 1
        lsa = (int(round(angs[-1] * 64)) << 1) | 1
        data = bytearray()
        for di in dists:
            w = int(di) << 2
            data += bytes([200, w & 0xFF, w >> 8])
        cs = TMiniPlus._checksum(0x55AA, ct, lsn, fsa, lsa, data, 3)
        return struct.pack("<HBBHHH", 0x55AA, ct, lsn, fsa, lsa, cs) + data

    out += pkt(1, [a[0]], [d[0]])                   # Start-Paket
    for i in range(1, len(a), 40):
        # gleichmäßig verteilte Winkel innerhalb des Pakets (wie beim echten LiDAR)
        seg_d = d[i:i + 40]
        seg_a = np.linspace(a[i], a[min(i + 39, len(a) - 1)], len(seg_d))
        out += pkt(0, list(seg_a), list(seg_d))
    return bytes(out)


def make_replay(path):
    rng = np.random.default_rng(0)
    tr = trajectory()
    data = [(i / 7.0, *sim_scan(p, rng)) for i, p in enumerate(tr)]
    with open(path, "wb") as f:
        pickle.dump(data, f)
    np.save(path + ".gt.npy", tr)
    print(f"Replay: {len(data)} Scans, Weg {np.sum(np.hypot(*np.diff(tr[:, :2], axis=0).T)):.1f} m")


if __name__ == "__main__":
    make_replay(sys.argv[1] if len(sys.argv) > 1 else "sim.pkl")
