"""Gemeinsame Hilfen für die Tests.

Die Simulatoren erzeugen ihre Aufnahmen einmal pro Testlauf in einen temporären
Ordner. Die Tests starten dann das echte main.py darauf (--replay --no-gui) und
messen die Genauigkeit gegen die Ground Truth der Simulation.
"""
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

TESTS = Path(__file__).resolve().parent
LIDAR = TESTS.parent
sys.path[:0] = [str(LIDAR), str(TESTS)]

ENV = dict(os.environ, PYTHONPATH=os.pathsep.join([str(LIDAR), str(TESTS)]))


def _sim(script, out, *args):
    subprocess.run([sys.executable, str(TESTS / script), str(out), *map(str, args)],
                   check=True, env=ENV, capture_output=True, text=True)
    return out


@pytest.fixture(scope="session")
def sims(tmp_path_factory):
    """Erzeugt Simulationen bei Bedarf (nur einmal pro Testlauf)."""
    d = tmp_path_factory.mktemp("sims")
    made = {}
    recipes = {
        "room":     ("sim_room.py", "room.pkl", ()),
        "hard90":   ("sim_hard.py", "hard90.pkl", ("--spin", 90)),
        "s3d":      ("sim3d.py", "s3d.pkl", ()),
        "tilt12":   ("sim_tilt.py", "tilt12.bin", ("--tilt", 12)),
        "beginner": ("sim_beginner.py", "beg.bin", ("--yaw-max", 400, "--seed", 2)),
    }

    def get(name):
        if name not in made:
            script, fn, args = recipes[name]
            made[name] = _sim(script, d / fn, *args)
        return made[name]
    return get


def run_main(replay, out_dir, *flags):
    """main.py auf einer Aufnahme laufen lassen -> (Dateipräfix, Textausgabe)."""
    prefix = Path(out_dir) / "k"
    r = subprocess.run([sys.executable, str(LIDAR / "main.py"), "--replay", str(replay), "--speed", "0",
                        "--no-gui", "--save", str(prefix), *flags],
                       env=ENV, capture_output=True, text=True, timeout=900)
    assert r.returncode == 0, r.stdout + r.stderr
    return str(prefix), r.stdout


def load_traj(csv):
    est = np.loadtxt(csv, delimiter=",", skiprows=1)
    return est[:, :3], est[:, 3].astype(int)


def align(est, gt_xy):
    """Beste starre 2D-Ausrichtung (Umeyama ohne Skalierung) -> (mu_e, R, mu_g, winkel)."""
    mu_e, mu_g = est[:, :2].mean(0), gt_xy.mean(0)
    H = (est[:, :2] - mu_e).T @ (gt_xy - mu_g)
    a = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return mu_e, R, mu_g, a


def ate(csv, gt_path):
    """Positionsfehler der geschätzten Bahn nach bester Ausrichtung -> (mittel_m, max_m, anteil_scans)."""
    gt = np.load(gt_path)
    est, idx = load_traj(csv)
    g = gt[idx]
    mu_e, R, mu_g, _ = align(est, g[:, :2])
    err = np.hypot(*(((est[:, :2] - mu_e) @ R.T + mu_g) - g[:, :2]).T)
    return err.mean(), err.max(), len(est) / len(gt)
