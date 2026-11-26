"""Genauigkeitstests auf simulierten Aufnahmen (dauern zusammen einige Minuten).

Gemessen wird gegen die Ground Truth des Simulators. Die Grenzen liegen mit
Abstand über den heute erreichten Werten (siehe Kommentare) - ein Test schlägt
also nur an, wenn eine Änderung die Kartierung spürbar verschlechtert.
"""
import math
import pickle

import numpy as np
import pytest

from conftest import align, ate, load_traj, run_main


def surface_hits(ply, to_world):
    """Anteil der 3D-Punkte, die < 10 cm neben einer echten Fläche aus sim3d liegen."""
    from sim3d import OBJ
    rows = open(ply).read().split("end_header\n")[1].split("\n")
    P = np.array([[float(v) for v in r.split()[:3]] for r in rows if r.strip()])
    xy, z = to_world(P[:, :2]), P[:, 2]
    best = np.full(len(P), np.inf)
    for (p1, p2), z0, z1 in OBJ:
        a, b = np.array(p1, float), np.array(p2, float)
        m = (z >= z0 - 0.05) & (z < z1 + 0.05)
        if m.any():
            ab = b - a
            t = np.clip(((xy[m] - a) @ ab) / (ab @ ab), 0, 1)
            best[m] = np.minimum(best[m], np.hypot(*(xy[m] - (a + t[:, None] * ab)).T))
    return np.mean(best < 0.10), len(P)


# ---------------------------------------------------------------- 2D

def test_2d_ruhiger_rundgang(sims, tmp_path):
    room = sims("room")
    prefix, _ = run_main(room, tmp_path, "--no-fc")
    mean, mx, frac = ate(prefix + "_trajectory.csv", str(room) + ".gt.npy")
    print(f"Raum: mittel {mean*100:.1f} cm, max {mx*100:.1f} cm, {frac:.0%} Scans")
    assert mean < 0.06 and mx < 0.20 and frac > 0.95


def test_2d_schnelle_drehungen(sims, tmp_path):
    """Hand-Rundgang mit 360°-Drehungen bei 90°/s und Verzerrung innerhalb des Scans."""
    rec = sims("hard90")
    prefix, _ = run_main(rec, tmp_path, "--no-fc")
    mean, mx, frac = ate(prefix + "_trajectory.csv", str(rec) + ".gt.npy")
    print(f"hart 90°/s: mittel {mean*100:.1f} cm, max {mx*100:.1f} cm, {frac:.0%} Scans")
    assert mean < 0.10 and mx < 0.35 and frac > 0.90


def test_2d_neigung_wird_ausgeglichen(sims, tmp_path):
    """Drohne kippt bis ±15°, steigt und sinkt (Pi-Aufzeichnung mit Lage + Höhenmesser)."""
    rec = sims("tilt12")
    prefix, out = run_main(rec, tmp_path, "--no-fc")
    mean, mx, frac = ate(prefix + "_trajectory.csv", str(rec) + ".gt.npy")
    print(f"Neigung 12°: mittel {mean*100:.1f} cm, max {mx*100:.1f} cm, {frac:.0%} Scans")
    # heute: ~3 cm; ~77 % der Scans (über 15° Neigung wird absichtlich übersprungen)
    assert mean < 0.10 and frac > 0.65


def test_2d_anfaengerflug_mit_fc(sims, tmp_path):
    """Hektischer Flug (bis 400°/s Gier). Mit FC-Gierwinkel bleibt die Bahn genau, ohne nicht."""
    rec = sims("beginner")
    prefix, out = run_main(rec, tmp_path / "fc")
    mean_fc, mx_fc, frac_fc = ate(prefix + "_trajectory.csv", str(rec) + ".gt.npy")
    assert "FC-Gierwinkel bei" in out, "FC-Daten wurden nicht genutzt"
    prefix, _ = run_main(rec, tmp_path / "nofc", "--no-fc")
    mean_no, _, frac_no = ate(prefix + "_trajectory.csv", str(rec) + ".gt.npy")
    print(f"Anfängerflug: mit FC {mean_fc*100:.1f} cm ({frac_fc:.0%}), ohne FC {mean_no*100:.1f} cm ({frac_no:.0%})")
    # heute: mit FC ~6 cm, ~77 % der Scans (Rest: Neigung über 15°); ohne FC über 1 m
    assert mean_fc < 0.15 and frac_fc > 0.65
    assert mean_fc < mean_no / 3, "FC-Gierwinkel bringt keinen Vorteil mehr"


# ---------------------------------------------------------------- Start-Fehler (Uni-Problem)

def make_bad_start(path):
    """Erst schnelle Drehung auf der Stelle mit unvollständigen ersten Scans, dann der normale Rundgang."""
    from sim_room import sim_scan, trajectory
    rng = np.random.default_rng(11)
    tr = trajectory()
    x0, y0, yaw0 = tr[0]
    spin = np.array([(x0, y0, yaw0 - math.radians(17) * (15 - k)) for k in range(15)])   # ~120°/s
    poses = np.vstack([spin, tr])
    data = []
    for i, p in enumerate(poses):
        a, d = sim_scan(p, rng)
        if i < 3:                                   # LiDAR läuft gerade an: nur ein Teil der Runde
            keep = a < 110
            a, d = a[keep], d[keep]
        data.append((i / 7.0, a, d))
    with open(path, "wb") as f:
        pickle.dump(data, f)
    return poses


def test_start_mit_drehung_und_teilscan(tmp_path):
    """Regression: früher blieb die Karte hier dauerhaft hängen (98 % der Scans verworfen)."""
    import slam
    from sources import replay_source
    path = tmp_path / "start.pkl"
    gt = make_bad_start(path)
    mp = slam.Mapper(use_fc=False)
    for t, ang, dist, inten, extra in replay_source(str(path)):
        mp.process(ang, dist, None)
    rejected = mp.n_rejected / mp.n_scans
    traj = np.array(mp.traj)
    print(f"Start: {rejected:.0%} verworfen, {len(traj)} Posen")
    assert rejected < 0.10
    # Bahn passt zur Wahrheit
    est_xy, g = traj[:, :2], gt[traj[:, 3].astype(int), :2]
    mu_e, R, mu_g, _ = align(est_xy, g)
    err = np.hypot(*((est_xy - mu_e) @ R.T + mu_g - g).T)
    assert np.median(err) < 0.10


# ---------------------------------------------------------------- 3D

def test_3d_schichten_vom_laptop(sims, tmp_path):
    """Laptop-Aufzeichnung auf 3 Höhen (Tisch, Fenster, Tür, Regal)."""
    rec = sims("s3d")
    prefix, _ = run_main(rec, tmp_path, "--3d")
    # Karte beginnt an der Startpose des Simulators (1, 3), Blick nach +x
    hits, n = surface_hits(prefix + ".ply", lambda xy: xy + np.array([1.0, 3.0]))
    print(f"3D Laptop: {hits:.0%} von {n} Punkten an echten Flächen")
    # heute: ~83 %
    assert hits > 0.75 and n > 2000


def test_3d_flug_mit_fc(sims, tmp_path):
    """Flug mit Neigung, Höhe vom Höhenmesser: Punkte müssen auf den echten Flächen liegen."""
    rec = sims("tilt12")
    prefix, _ = run_main(rec, tmp_path, "--3d")
    gt = np.load(str(rec) + ".gt.npy")
    est, idx = load_traj(prefix + "_2d_trajectory.csv")
    mu_e, R, mu_g, _ = align(est, gt[idx, :2])
    hits, n = surface_hits(prefix + ".ply", lambda xy: (xy - mu_e) @ R.T + mu_g)
    print(f"3D Flug: {hits:.0%} von {n} Punkten an echten Flächen")
    assert hits > 0.80 and n > 2000


# ---------------------------------------------------------------- Aufzeichnen / Abspielen

@pytest.mark.parametrize("flags", [(), ("--3d",)], ids=["2d", "3d"])
def test_aufzeichnung_wieder_abspielbar(sims, tmp_path, flags):
    """--record schreibt ein .pkl, das wieder genau dieselbe Karte ergibt."""
    room = sims("room")
    rec = tmp_path / "roh.pkl"
    p1, _ = run_main(room, tmp_path / "a", "--record", str(rec), *flags)
    p2, _ = run_main(rec, tmp_path / "b", *flags)
    suffix = "_2d_trajectory.csv" if flags else "_trajectory.csv"
    assert open(p1 + suffix).read() == open(p2 + suffix).read()
    with open(rec, "rb") as f:
        assert all(len(item) == 5 for item in pickle.load(f))
