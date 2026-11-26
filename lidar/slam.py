"""Positionsschätzung und Kartenaufbau (2D-SLAM ohne Loop Closure).

Pro Scan: Vorhersage (letzte Bewegung bzw. FC-Gierwinkel) -> Entzerrung ->
Korrelationssuche -> Point-to-Line-ICP -> Belegungskarte aktualisieren.
"""
import math

import cv2
import numpy as np
from scipy.spatial import cKDTree

from config import *  # noqa: F401,F403
from maps import GridMap

def tilt_deg(sens):
    if not sens or sens.get("roll") is None:
        return None
    return math.degrees(math.acos(max(-1.0, min(1.0, math.cos(sens["roll"]) * math.cos(sens["pitch"])))))


def sensor_height(sens):
    """Höhe des LiDARs über dem Boden: Höhenmesser (mit Neigung), sonst EKF, sonst None."""
    if not sens:
        return None
    if sens.get("range") is not None and sens["range"] > 0.02:
        c = 1.0
        if sens.get("roll") is not None:
            c = math.cos(sens["roll"]) * math.cos(sens["pitch"])
        return sens["range"] * c + LIDAR_ABOVE_RANGEFINDER
    if sens.get("ekf_z") is not None:
        return -sens["ekf_z"]
    return None


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, d):
    """Pose a ⊕ relative Bewegung d (d im Koordinatensystem von a)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * d[0] - s * d[1], a[1] + s * d[0] + c * d[1], wrap(a[2] + d[2])])


def relative(a, b):
    """Bewegung von Pose a nach Pose b, im Koordinatensystem von a."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])])


def transform(pts, pose):
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    return pts @ np.array([[c, s], [-s, c]]) + (x, y)


def scan_to_points(ang_deg, dist_mm):
    """-> Punkte im LiDAR-System und Zeitanteil (0..1) innerhalb der Umdrehung."""
    n = len(ang_deg)
    frac = np.arange(n) / max(n - 1, 1)        # Punkte kommen in Aufnahmereihenfolge
    r = dist_mm / 1000.0
    ok = (r > MIN_RANGE) & (r < MAX_RANGE)
    th = np.radians(ang_deg[ok])
    if not FLIP:
        th = -th                                # Uhrzeigersinn -> mathematisch positiv
    th = th + math.radians(LIDAR_YAW_OFFSET_DEG)
    return np.column_stack((r[ok] * np.cos(th), r[ok] * np.sin(th))), frac[ok]


def level_points(pts, roll, pitch):
    """Geneigten Scan in die Waagerechte umrechnen.

    pts: Punkte in der LiDAR-Ebene (x vorne, y links). roll/pitch wie ArduPilot
    (roll: rechts runter positiv, pitch: Nase hoch positiv).
    -> (Punkte waagerecht projiziert (N,2), Höhe jedes Punkts relativ zum LiDAR (N,))
    """
    xf, yf = pts[:, 0], -pts[:, 1]              # in Fahrzeug-Koordinaten vorne/rechts/unten
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    n = cp * xf + sp * sr * yf                  # R = Ry(pitch) * Rx(roll)
    e = cr * yf
    d = -sp * xf + cp * sr * yf
    return np.column_stack((n, -e)), -d


def deskew(pts, frac, vel):
    """Punkte auf die Pose am Scan-Ende umrechnen (vel = Bewegung pro Scan)."""
    if not DESKEW:
        return pts
    k = frac - 1.0                              # <= 0: so viele Scans vor dem Ende
    th = k * vel[2]
    c, s = np.cos(th), np.sin(th)
    x = c * pts[:, 0] - s * pts[:, 1] + k * vel[0]
    y = s * pts[:, 0] + c * pts[:, 1] + k * vel[1]
    return np.column_stack((x, y))


def voxel_downsample(pts, size):
    keys = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


class Reference:
    """Wandpunkte der Karte um eine Position: KD-Baum, Normalen, Wahrscheinlichkeitsfeld."""

    def __init__(self, grid, center, radius):
        self.ok = False
        cx, cy = grid.to_cell(np.array(center[:2]))
        r = int(radius / grid.res)
        n = grid.n
        self.x0, self.y0 = max(cx - r, 0), max(cy - r, 0)
        x1, y1 = min(cx + r, n), min(cy + r, n)
        win = grid.L[self.y0:y1, self.x0:x1]
        occ = win > L_OCC_THRESH
        self.known = np.abs(win) > 0.35          # schon beobachtet (frei oder Wand)
        if occ.sum() < MIN_MAP_POINTS:           # Startphase: einmal gesehene Wände reichen
            occ = win > START_THRESH
        ys, xs = np.nonzero(occ)
        if len(xs) < MIN_MAP_POINTS:
            return
        self.res, self.origin = grid.res, grid.origin
        self.pts = np.column_stack(((xs + self.x0 + 0.5) * grid.res + grid.origin,
                                    (ys + self.y0 + 0.5) * grid.res + grid.origin))
        self.tree = cKDTree(self.pts)
        # Normalen über die Nachbarn (PCA)
        _, nn = self.tree.query(self.pts, k=min(9, len(self.pts)))
        nb = self.pts[nn] - self.pts[nn].mean(1, keepdims=True)
        cov = np.einsum("mki,mkj->mij", nb, nb)
        w, v = np.linalg.eigh(cov)
        self.normals = v[:, :, 0]
        self.weight = np.where(w[:, 0] < 0.2 * w[:, 1], 1.0, 0.3)  # Ecken weniger stark
        # Wahrscheinlichkeitsfeld für die Korrelationssuche
        dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * grid.res
        self.field = np.exp(-0.5 * (dist / FIELD_SIGMA) ** 2).astype(np.float32)
        self.ok = True

    def search(self, scan, pred, yaw_range, xy_range):
        """Korrelationssuche über Drehung und Verschiebung. -> (pose, score 0..1)."""
        H, W = self.field.shape
        k = int(round(xy_range / self.res))
        offs = np.arange(-k, k + 1)
        yaws = np.radians(np.arange(-yaw_range, yaw_range + 1e-6, SEARCH_YAW_STEP_DEG))
        # leichte Bevorzugung der Vorhersage, damit symmetrische Räume nicht springen
        prior_xy = np.exp(-0.5 * ((offs[:, None] ** 2 + offs[None, :] ** 2) * self.res ** 2)
                          / (0.6 * xy_range) ** 2)
        best, best_pose = -1.0, pred
        for dy in yaws:
            th = pred[2] + dy
            w = transform(scan, (pred[0], pred[1], th))
            cx = np.floor((w[:, 0] - self.origin) / self.res).astype(np.int64) - self.x0
            cy = np.floor((w[:, 1] - self.origin) / self.res).astype(np.int64) - self.y0
            CX = cx[:, None, None] + offs[None, None, :]
            CY = cy[:, None, None] + offs[None, :, None]
            valid = (CX >= 0) & (CX < W) & (CY >= 0) & (CY < H)
            vals = self.field[np.clip(CY, 0, H - 1), np.clip(CX, 0, W - 1)] * valid
            score = vals.mean(0) * (0.85 + 0.15 * prior_xy) \
                * (0.9 + 0.1 * math.exp(-0.5 * (dy / math.radians(0.6 * yaw_range)) ** 2))
            iy, ix = np.unravel_index(int(score.argmax()), score.shape)
            if score[iy, ix] > best:
                best = float(score[iy, ix])
                best_pose = np.array([pred[0] + offs[ix] * self.res, pred[1] + offs[iy] * self.res, wrap(th)])
        return best_pose, best

    def icp(self, scan, init):
        """Point-to-Line-ICP mit Huber-Gewichten. -> (pose, inlier_anteil in bekannten Bereichen)."""
        pose = np.array(init, dtype=float)
        for max_d in ICP_MAX_DIST:
            for _ in range(ICP_ITER):
                w = transform(scan, pose)
                d, idx = self.tree.query(w, distance_upper_bound=max_d)
                m = np.isfinite(d)
                if m.sum() < 30:
                    return pose, 0.0
                p, i = w[m], idx[m]
                n = self.normals[i]
                r = np.einsum("ij,ij->i", n, p - self.pts[i])
                perp = np.column_stack((-(p[:, 1] - pose[1]), p[:, 0] - pose[0]))
                J = np.column_stack((n, np.einsum("ij,ij->i", n, perp)))
                ar = np.abs(r)
                wts = np.where(ar < HUBER, 1.0, HUBER / np.maximum(ar, 1e-9)) * self.weight[i]
                A = J.T @ (J * wts[:, None]) + np.diag([1e-4, 1e-4, 1e-4])
                dx = np.linalg.solve(A, -J.T @ (wts * r))
                pose = np.array([pose[0] + dx[0], pose[1] + dx[1], wrap(pose[2] + dx[2])])
                if math.hypot(dx[0], dx[1]) < 1e-4 and abs(dx[2]) < 1e-5:
                    break
        return pose, self.quality(scan, pose)

    def quality(self, scan, pose):
        """Anteil der Punkte nahe einer Wand - nur unter Punkten in bekannten Zellen."""
        w = transform(scan, pose)
        d, _ = self.tree.query(w, distance_upper_bound=0.10)
        inl = np.isfinite(d)
        cx = np.floor((w[:, 0] - self.origin) / self.res).astype(np.int64) - self.x0
        cy = np.floor((w[:, 1] - self.origin) / self.res).astype(np.int64) - self.y0
        H, W = self.known.shape
        inside = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
        known = np.zeros(len(w), bool)
        known[inside] = self.known[cy[inside], cx[inside]]
        known |= inl                             # Treffer an Wänden zählen immer
        if known.sum() < MIN_KNOWN_POINTS:
            return 0.0
        return float(inl.sum() / known.sum())


def update_wall_cache(grid, center, radius):
    """Für sichere Wandzellen bestimmen: Teil einer geraden Fläche (>= 40 cm)? + Normale."""
    cx, cy = grid.to_cell(np.array(center[:2]))
    r = int(radius / grid.res)
    x0, y0 = max(cx - r, 0), max(cy - r, 0)
    x1, y1 = min(cx + r, grid.n), min(cy + r, grid.n)
    ys, xs = np.nonzero(grid.solid[y0:y1, x0:x1])
    grid.wall_lin[y0:y1, x0:x1] = False
    if len(xs) < 8:
        return
    pts = np.column_stack(((xs + x0 + 0.5) * grid.res, (ys + y0 + 0.5) * grid.res))
    tree = cKDTree(pts)
    for i, nb in enumerate(tree.query_ball_point(pts, MIRROR_WALL_RADIUS)):
        if len(nb) < 8:
            continue
        q = pts[nb] - pts[nb].mean(0)
        w, v = np.linalg.eigh(q.T @ q)
        span = 2.0 * math.sqrt(3.0 * w[1] / len(nb))       # Länge des Abschnitts
        if w[0] < 0.08 * w[1] and span >= MIRROR_WALL_MIN_LEN:
            grid.wall_lin[ys[i] + y0, xs[i] + x0] = True
            grid.wall_n[ys[i] + y0, xs[i] + x0] = v[:, 0]


def filter_reflections(grid, ref, pose, world):
    """Spiegelbilder erkennen und den Strahl an der Spiegelfläche enden lassen.

    Ein Spiegel wird frontal als Wand gesehen, schräg wirft er den Strahl weiter.
    Regel: Läuft ein Strahl durch eine Zelle, die irgendwann sicher als gerade Wand
    (>= 40 cm) erkannt wurde, und liegt der Punkt deutlich dahinter, ist er ein
    Spiegelbild - sofern das für eine zusammenhängende Folge von Strahlen gilt.
    -> (neue Weltpunkte, Anzahl erkannter Spiegelpunkte)
    """
    if len(world) == 0 or not grid.wall_lin.any():
        return world, 0
    o = np.array(pose[:2])
    vec = world - o
    rng = np.hypot(vec[:, 0], vec[:, 1])
    dirs = vec / np.maximum(rng[:, None], 1e-9)
    steps = np.arange(0.2, max(rng.max() - MIRROR_BEHIND_MIN, 0.21), grid.res * 0.5)
    samples = o + dirs[:, None, :] * steps[None, :, None]
    valid = steps[None, :] < (rng[:, None] - MIRROR_BEHIND_MIN)
    cells = grid.to_cell(samples.reshape(-1, 2)).reshape(len(world), len(steps), 2)
    cxs = np.clip(cells[..., 0], 0, grid.n - 1)
    cys = np.clip(cells[..., 1], 0, grid.n - 1)
    S = grid.wall_lin[cys, cxs] & valid
    cand = np.nonzero(S.any(1))[0]
    if len(cand) < MIRROR_MIN_RUN:
        return world, 0
    first = S[cand].argmax(1)
    fx, fy = cxs[cand, first], cys[cand, first]
    b = o + dirs[cand] * steps[first][:, None]
    n = grid.wall_n[fy, fx].astype(np.float64)
    n[np.einsum("ij,ij->i", n, o - b) < 0] *= -1         # Normale zum Sensor hin
    p = world[cand]
    behind = -np.einsum("ij,ij->i", p - b, n)
    inc_ok = np.abs(np.einsum("ij,ij->i", dirs[cand], n)) > math.sin(math.radians(MIRROR_MIN_INC))
    geo_ok = inc_ok & (behind >= MIRROR_BEHIND_MIN)
    if MIRROR_CHECK_REFLECTION and ref.ok:
        dr, _ = ref.tree.query(p + 2 * behind[:, None] * n)
        confirm = geo_ok & (dr < MIRROR_CONFIRM + 0.04 * behind)
    else:
        confirm = geo_ok

    ghost = np.zeros(len(cand), bool)
    ok_idx = np.nonzero(geo_ok)[0]
    if len(ok_idx):
        gaps = np.nonzero(np.diff(cand[ok_idx]) > 3)[0]
        for run in np.split(ok_idx, gaps + 1):
            if len(run) >= MIRROR_MIN_RUN and confirm[run].mean() >= 0.5:
                ghost[run] = True
    if not ghost.any():
        return world, 0
    out = world.copy()
    g = cand[ghost]
    t = np.einsum("ij,ij->i", b[ghost] - o, n[ghost]) / np.einsum("ij,ij->i", dirs[g], n[ghost])
    out[g] = o + dirs[g] * np.maximum(t, 0.0)[:, None]  # Strahl endet an der Spiegelfläche
    return out, int(ghost.sum())


class Mapper:
    """Schätzt pro Scan die Position und baut die 2D-Karte auf.

    use_fc: Gierwinkel des Flugcontrollers (falls in den Sensordaten) als Vorhersage
    für die Drehung zwischen zwei Scans nutzen. Ohne FC-Daten ohne Wirkung.
    """

    def __init__(self, use_fc=USE_FC):
        self.use_fc = use_fc
        self.fc_yaw_ref = None                  # FC-Gierwinkel beim Scan der aktuellen Pose
        self.n_fc_used = 0
        self.n_fc_distrust = 0
        self.grid = GridMap(MAP_SIZE_M, RESOLUTION)
        self.pose = np.zeros(3)
        self.vel = np.zeros(3)                  # Bewegung pro Scan (im Fahrzeugsystem)
        self.lost = 0
        self.traj = []
        self.n_scans = self.n_rejected = 0
        self.n_mirror = 0                       # erkannte Spiegelpunkte (gesamt)
        self.n_tilt = 0                         # wegen zu starker Neigung verworfen
        self.last_zrel = None                   # Höhe der Punkte relativ zum LiDAR (bei Neigung)
        self.n_reanchor = 0                     # nach langem Verlust mit gelockerter Schwelle angenommen
        self.last_tilt = None
        self.last_score = self.last_ratio = 1.0

    def _match(self, ref, pts, frac, pred, wide):
        """Korrelationssuche + ICP, 2 Durchgänge (Entzerrung mit neuer Geschwindigkeit)."""
        w = 1 if wide else 0
        vel = self.vel.copy()
        pose, ratio = pred, 0.0
        for it in range(2):
            pd = deskew(pts, frac, vel)
            if it == 0:
                coarse = voxel_downsample(pd, 0.10)
                pose, self.last_score = ref.search(coarse, pred, SEARCH_YAW_DEG[w], SEARCH_XY_M[w])
            pose, ratio = ref.icp(voxel_downsample(pd, RESOLUTION), pose)
            vel = relative(self.pose, pose)
        return pose, ratio, vel

    def process(self, ang, dist, sens=None):
        """Einen Scan verarbeiten (mit FC-Gierwinkel als Vorhersage, falls aktiv und vorhanden).

        -> Weltpunkte des Scans (für die Anzeige).
        """
        yaw_fc = sens.get("yaw") if (self.use_fc and sens) else None
        used = False
        d = 0.0
        if yaw_fc is not None and self.fc_yaw_ref is not None:
            # ArduPilot: Gier im Uhrzeigersinn (NED) -> Karte: gegen den Uhrzeigersinn
            d = -wrap(yaw_fc - self.fc_yaw_ref)
            self.vel = np.array([self.vel[0], self.vel[1], d])
            used = True
        pose_before = self.pose.copy()
        traj_before = len(self.traj)
        world = self._process_core(ang, dist, sens)
        if used:
            self.n_fc_used += 1
            if len(self.traj) > traj_before:      # angenommen: FC mit Scan-Matching vergleichen
                d_match = wrap(self.pose[2] - pose_before[2])
                if abs(math.degrees(wrap(d_match - d))) > FC_MAX_DISAGREE_DEG:
                    self.n_fc_distrust += 1       # z. B. Kompass-Sprung: Scan-Matching gewinnt
        if yaw_fc is not None and not np.array_equal(self.pose, pose_before):
            self.fc_yaw_ref = yaw_fc              # Bezug für den nächsten Scan
        elif yaw_fc is not None and self.fc_yaw_ref is None:
            self.fc_yaw_ref = yaw_fc
        return world

    def _process_core(self, ang, dist, sens=None):
        """Einen Scan verarbeiten. sens = Lage/Höhe vom FC (optional).

        -> Weltpunkte des Scans (für die Anzeige). Bei akzeptiertem Scan steht in
        self.last_zrel die Höhe jedes Punkts relativ zum LiDAR (oder None).
        """
        pts, frac = scan_to_points(ang, dist)
        idx = self.n_scans
        self.n_scans += 1
        self.last_zrel = None
        zrel = None
        self.last_tilt = tilt_deg(sens)
        if TILT_COMP and self.last_tilt is not None:
            if self.last_tilt > MAX_TILT_DEG:
                self.n_tilt += 1
                return np.zeros((0, 2))
            pts, zrel = level_points(pts, sens["roll"], sens["pitch"])
            h = sensor_height(sens)
            if h is not None:                   # Bodentreffer des geneigten Strahls entfernen
                keep = h + zrel > FLOOR_CLEARANCE
                pts, frac, zrel = pts[keep], frac[keep], zrel[keep]
        if len(pts) < 50:
            return np.zeros((0, 2))
        pred = compose(self.pose, self.vel)
        ref = Reference(self.grid, pred, MAX_RANGE + 1.5)

        if not ref.ok:                          # Karte noch leer: einfach eintragen
            if len(pts) < MIN_SCAN_POINTS:      # unvollständiger Scan: nicht als Start nehmen
                return transform(pts, pred)
            world = transform(deskew(pts, frac, self.vel), pred)
            self.grid.update(pred, world)
            self.pose = pred
            self.traj.append([*pred, idx])
            self.last_zrel = zrel
            return world

        pose, ratio, vel = self._match(ref, pts, frac, pred, wide=bool(self.lost))
        if ratio < RETRY_BELOW and not self.lost:   # unsicher -> zusätzlich weiträumig suchen
            pose2, ratio2, vel2 = self._match(ref, pts, frac, pred, wide=True)
            if ratio2 > ratio + 0.02:
                pose, ratio, vel = pose2, ratio2, vel2
        self.last_ratio = ratio

        relaxed = self.lost > RELOC_AFTER and ratio >= RELAXED_RATIO
        if relaxed:
            self.n_reanchor += 1
        if ratio >= MIN_INLIER_RATIO or relaxed:
            self.vel = 0.7 * vel + 0.3 * self.vel if not self.lost else vel
            self.pose = pose
            self.lost = 0
            world = transform(deskew(pts, frac, vel), pose)
            if MIRROR_FILTER:
                if idx % MIRROR_CACHE_EVERY == 0:
                    update_wall_cache(self.grid, pose, MAX_RANGE + 1.0)
                world, ng = filter_reflections(self.grid, ref, pose, world)
                self.n_mirror += ng
            self.grid.update(pose, world)
            self.traj.append([*pose, idx])
            self.last_zrel = zrel
            return world

        self.n_rejected += 1                    # Scan passt nicht: nicht in die Karte
        self.lost += 1
        self.pose = pose if ratio > 0.3 else pred
        self.vel *= 0.5
        return transform(pts, self.pose)
