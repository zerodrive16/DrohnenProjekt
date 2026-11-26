"""Karten: 2D-Belegungskarte, 3D-Schichten, Speichern (PNG/YAML/NPY/PLY)."""
import math
import time
from pathlib import Path

import cv2
import numpy as np

from config import *  # noqa: F401,F403

def output_paths(save, record, default_name):
    """Ausgabeort festlegen.

    save ohne Ordner (oder leer) -> karten/<datum_zeit>/<save>, sonst genau dort.
    record ohne Ordner -> in denselben Ordner wie die Karte.
    -> (datei_präfix, record_pfad|None)
    """
    if save and Path(save).parent != Path("."):
        prefix = Path(save)
    else:
        prefix = KARTEN_DIR / time.strftime("%Y%m%d_%H%M%S") / (save or default_name)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    if record and Path(record).parent == Path("."):
        record = str(prefix.parent / record)
    print(f"Ausgabe: {prefix.parent}")
    return str(prefix), record


class GridMap:
    def __init__(self, size_m, res):
        self.res = res
        self.n = int(round(size_m / res))
        self.origin = -size_m / 2.0             # Weltkoordinate der Zelle (0, 0)
        self.L = np.zeros((self.n, self.n), dtype=np.float32)
        self.solid = np.zeros((self.n, self.n), dtype=bool)   # irgendwann sicher als Wand erkannt
        self.wall_lin = np.zeros((self.n, self.n), dtype=bool)  # ... und Teil einer geraden Fläche
        self.wall_n = np.zeros((self.n, self.n, 2), dtype=np.float32)

    def to_cell(self, xy):
        return np.floor((xy - self.origin) / self.res).astype(np.int64)

    def update(self, pose, pts_world):
        o = np.array(pose[:2])
        vec = pts_world - o
        rng = np.hypot(vec[:, 0], vec[:, 1])
        dirs = vec / rng[:, None]
        steps = np.arange(0.0, rng.max(), self.res * 0.7)
        samples = o + dirs[:, None, :] * steps[None, :, None]
        mask = steps[None, :] < (rng[:, None] - self.res)
        free = self._flat(self.to_cell(samples[mask]))
        hits = self._flat(self.to_cell(pts_world))
        free = np.setdiff1d(np.unique(free), hits)
        hits = np.unique(hits)
        L = self.L.reshape(-1)
        L[free] += L_FREE
        L[hits] += L_OCC
        np.clip(L, L_MIN, L_MAX, out=L)
        sure = hits[L[hits] >= L_WALL]
        self.solid.reshape(-1)[sure] = True

    def _flat(self, cells):
        ok = (cells[:, 0] >= 0) & (cells[:, 0] < self.n) & (cells[:, 1] >= 0) & (cells[:, 1] < self.n)
        c = cells[ok]
        return c[:, 1] * self.n + c[:, 0]      # Zeile = y, Spalte = x

    def image(self, L=None):
        L = self.L if L is None else L
        img = np.full(L.shape, 128, np.uint8)  # unbekannt = grau
        p = 1.0 - 1.0 / (1.0 + np.exp(L))
        known = np.abs(L) >= 0.05
        img[known] = (255 * (1.0 - p[known])).astype(np.uint8)
        return np.flipud(img)                   # y nach oben

    def save(self, prefix, trajectory):
        known = np.abs(self.L) >= 0.05
        if known.any():                         # auf den erfassten Bereich zuschneiden
            ys, xs = np.nonzero(known)
            m = 20
            y0, y1 = max(ys.min() - m, 0), min(ys.max() + m + 1, self.n)
            x0, x1 = max(xs.min() - m, 0), min(xs.max() + m + 1, self.n)
        else:
            y0, y1, x0, x1 = 0, self.n, 0, self.n
        Lc = self.L[y0:y1, x0:x1]
        cv2.imwrite(prefix + ".png", self.image(Lc))
        np.save(prefix + ".npy", Lc)
        name = prefix.replace("\\", "/").split("/")[-1]
        with open(prefix + ".yaml", "w") as f:  # ROS-map_server-Format
            f.write(f"image: {name}.png\nresolution: {self.res}\n"
                    f"origin: [{self.origin + x0 * self.res:.3f}, {self.origin + y0 * self.res:.3f}, 0.0]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        if trajectory:
            np.savetxt(prefix + "_trajectory.csv", np.array(trajectory), delimiter=",",
                       header="x,y,yaw,scan_idx", comments="", fmt=["%.4f", "%.4f", "%.5f", "%d"])
        print(f"Gespeichert: {prefix}.png ({x1 - x0}x{y1 - y0} px) / .yaml / .npy / _trajectory.csv")


class LayerMap:
    """Eine Belegungskarte pro Höhenschicht (gleiche Koordinaten wie die 2D-Karte)."""

    def __init__(self):
        self.layers = {}                         # schicht_index -> GridMap

    def update(self, z, pose, pts_world, zrel=None):
        """z = Höhe des LiDARs, zrel = Höhe jedes Punkts relativ zum LiDAR (bei Neigung)."""
        zp = np.full(len(pts_world), z) if zrel is None else z + zrel
        ks = np.floor(zp / LAYER_H + 1e-9).astype(int)
        for k in np.unique(ks):
            m = ks == k
            if m.sum() < 5:
                continue
            if k not in self.layers:
                self.layers[k] = GridMap(MAP_SIZE_M, RESOLUTION)
                print(f"Neue Schicht: {k * LAYER_H:.2f} - {(k + 1) * LAYER_H:.2f} m")
            self.layers[k].update(pose, pts_world[m])

    def points(self):
        """-> (N,3) Punkte aller belegten Zellen, z = Schichtmitte."""
        out = []
        for k, g in sorted(self.layers.items()):
            ys, xs = np.nonzero(g.L > MIN_HITS_LOGODDS)
            if len(xs) == 0:
                continue
            x = (xs + 0.5) * g.res + g.origin
            y = (ys + 0.5) * g.res + g.origin
            out.append(np.column_stack((x, y, np.full(len(xs), (k + 0.5) * LAYER_H))))
        return np.vstack(out) if out else np.zeros((0, 3))

    def save(self, prefix):
        pts = self.points()
        if len(pts) == 0:
            print("Keine 3D-Punkte zum Speichern.")
            return
        z = pts[:, 2]
        span = max(z.max() - z.min(), 1e-6)
        rgb = cv2.applyColorMap(((z - z.min()) / span * 255).astype(np.uint8)[:, None],
                                cv2.COLORMAP_TURBO)[:, 0, ::-1]
        with open(prefix + ".ply", "w") as f:
            f.write("ply\nformat ascii 1.0\n"
                    f"element vertex {len(pts)}\n"
                    "property float x\nproperty float y\nproperty float z\n"
                    "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                    "end_header\n")
            for (x, y, zz), (r, g, b) in zip(pts, rgb):
                f.write(f"{x:.3f} {y:.3f} {zz:.3f} {r} {g} {b}\n")
        for k, g in self.layers.items():         # jede Schicht auch als Bild
            h = k * LAYER_H
            g.save(f"{prefix}_schicht_{'m' if h < 0 else ''}{abs(h):.2f}m", [])
        print(f"3D-Punktwolke: {prefix}.ply ({len(pts)} Punkte, {len(self.layers)} Schichten)")
