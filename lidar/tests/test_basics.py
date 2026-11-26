"""Schnelle Tests: LiDAR-Parser, Funkprotokoll Pi<->Laptop, Pi-Aufzeichnungen, Einstellungen."""
import math

import numpy as np

import config
import protocol as P
import sources
from sim_room import build_packets, sim_scan


def test_parser_liest_lidar_bytes():
    """Echte YDLIDAR-Pakete, in zufälligen Stücken gefüttert, mit Müll davor."""
    rng = np.random.default_rng(1)
    dev = sources.TMiniPlus.__new__(sources.TMiniPlus)   # ohne serielle Schnittstelle
    dev.buf, dev.sample_size, dev.check_cs = bytearray(), None, True
    dev._tried = dev.cs_errors = 0
    dev._a, dev._d, dev._i = [], [], []
    scans = [sim_scan((5, 2, 0.3), rng) for _ in range(3)]
    stream = b"\x13\x37garbage" + b"".join(build_packets(a, d) for a, d in scans)
    stream += build_packets(*scans[0])[:200]          # nächstes Start-Paket schließt Scan 3 ab
    got, pos = [], 0
    while pos < len(stream):
        n = int(rng.integers(1, 700))
        dev.buf += stream[pos:pos + n]
        pos += n
        got.extend(dev._parse())
    assert len(got) == 3
    assert dev.sample_size == 3
    for (a, d), (ga, gd, gi) in zip(scans, got):
        assert np.all(gi == 200)
        assert len(ga) == len(a)
        assert np.max(np.abs(gd - d)) <= 1
        assert np.max(np.abs((ga - a + 180) % 360 - 180)) < 0.5


def test_protokoll_scan_in_teilen():
    """Ein Scan mit 620 Punkten wird in Teile zerlegt und wieder zusammengesetzt."""
    a = np.linspace(0, 359.5, 620)
    d = np.arange(620) * 10.0
    i = np.full(620, 77)
    msgs = P.encode_scan(42, 12.5, a, d, i)
    assert len(msgs) == 4                              # je höchstens 200 Punkte
    assert all(len(m) < 1400 for m in msgs)            # passt in ein UDP-Paket ohne Fragmentierung
    asm = P.ScanAssembler()
    out = [asm.add(t, data) for t, data in ((P.decode(m)[1], P.decode(m)[2]) for m in msgs[::-1])]
    t, A, D, I = out[-1]
    assert out[:-1] == [None] * 3 and t == 12.5
    assert np.allclose(A, a, atol=0.01) and D == list(d.astype(int)) and set(I) == {77}


def test_protokoll_verlorene_teile():
    """Fehlt ein Teil, wird der Scan verworfen und gezählt - der nächste kommt trotzdem durch."""
    asm = P.ScanAssembler()
    a, d, i = np.arange(300.0), np.full(300, 1000.0), np.zeros(300)
    m1 = P.encode_scan(1, 1.0, a, d, i)
    m2 = P.encode_scan(2, 1.2, a, d, i)
    assert asm.add(*P.decode(m1[0])[1:]) is None      # Teil 2 von Scan 1 geht verloren
    for m in m2:
        res = asm.add(*P.decode(m)[1:])
    assert res is not None and len(res[1]) == 300
    assert asm.lost == 1 and asm.complete == 1


def test_protokoll_lage_und_hoehe():
    t, typ = 3.0, None
    typ, tt, pose = P.decode(P.encode_pose(t, 0.1, -0.2, 1.5, math.nan, math.nan, -1.2))
    assert typ == P.T_POSE and tt == t and np.allclose(pose[:3], (0.1, -0.2, 1.5)) and math.isclose(pose[5], -1.2, rel_tol=1e-6)
    typ, tt, rng = P.decode(P.encode_range(t, 1.25, 25))
    assert typ == P.T_RANGE and math.isclose(rng[0], 1.25, rel_tol=1e-6) and rng[1] == 25
    assert P.decode(b"XX" + bytes(20)) is None


def test_pi_aufzeichnung_mit_fc_daten(tmp_path):
    """.bin wie vom Pi: Scans bekommen die Lage/Höhe zum passenden Zeitpunkt."""
    path = tmp_path / "log.bin"
    msgs = []
    for k in range(40):                                 # 20 Hz Lage + Höhenmesser
        t = k * 0.05
        msgs.append(P.encode_pose(t, 0.05, -0.03, 0.01 * k, math.nan, math.nan, math.nan))
        msgs.append(P.encode_range(t, 1.0 + 0.01 * k, 25))
    for s in range(10):                                 # 6 Hz Scans
        msgs += P.encode_scan(s, 0.16 * (s + 1), np.arange(0, 360, 1.0), np.full(360, 2000.0), np.full(360, 100))
    msgs.sort(key=lambda m: P.decode(m)[1])
    with open(path, "wb") as f:
        for m in msgs:
            f.write(P.LEN.pack(len(m)) + m)
    out = list(sources.replay_source(str(path)))
    assert len(out) == 10
    t, ang, dist, inten, sens = out[3]
    assert len(ang) == 360 and isinstance(sens, dict)
    assert math.isclose(sens["roll"], 0.05, rel_tol=1e-5)
    k = round((t - sources.SCAN_MID) / 0.05)            # Lage aus der Scanmitte
    assert math.isclose(sens["yaw"], 0.01 * k, abs_tol=0.011)
    assert sens["range"] is not None


def test_wichtige_einstellungen():
    """Schutz gegen versehentlich verstellte Einstellungen (ist schon einmal passiert)."""
    assert config.TILT_COMP is True, "Neigungsausgleich muss an sein"
    assert config.USE_FC is True
    assert config.MIN_MAP_POINTS > config.MIN_KNOWN_POINTS, "sonst sperrt sich der Start aus"
    assert config.RELAXED_RATIO < config.MIN_INLIER_RATIO


def test_ausgabeordner(tmp_path, monkeypatch):
    import maps
    monkeypatch.setattr(maps, "KARTEN_DIR", tmp_path / "karten")
    prefix, rec = maps.output_paths(None, "roh.pkl", "karte")
    assert prefix.startswith(str(tmp_path / "karten")) and prefix.endswith("karte")
    assert rec.startswith(str(tmp_path / "karten")) and rec.endswith("roh.pkl")
    prefix, _ = maps.output_paths(str(tmp_path / "x" / "meine"), None, "karte")
    assert prefix == str(tmp_path / "x" / "meine")
