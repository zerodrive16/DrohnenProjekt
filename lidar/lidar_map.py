"""2D-Karte mit dem YDLIDAR T-mini Plus - direkt am Laptop (USB).

Die Position wird nur aus den LiDAR-Scans geschätzt (kein Flugcontroller, kein GPS):
  1. Bewegungsvorhersage (letzte Geschwindigkeit wird fortgeschrieben)
  2. Entzerrung: jeder Punkt wird auf das Scan-Ende zurückgerechnet
     (der LiDAR bewegt sich während einer Umdrehung mit)
  3. Korrelationssuche: viele Dreh-/Verschiebungs-Kandidaten gegen die Karte
     -> findet die richtige Lage auch bei schnellen Drehungen
  4. Feinabgleich: Point-to-Line-ICP mit robusten Gewichten
  5. Verliert es den Anschluss, wird beim nächsten Scan weiträumig gesucht

Installation:
    pip install pyserial numpy scipy opencv-python

Start:
    python lidar_map.py                       # COM-Port wird automatisch gesucht
    python lidar_map.py --port COM5           # Port selbst angeben
    python lidar_map.py --record test.pkl     # zusätzlich Rohdaten aufzeichnen
    python lidar_map.py --replay test.pkl     # Aufzeichnung abspielen (ohne LiDAR)
    python lidar_map.py --replay test.pkl --speed 0   # so schnell wie möglich
    python lidar_map.py --pi 100.83.11.79     # LiDAR hängt am Pi (pi_streamer.py läuft dort)
    python lidar_map.py --replay scan_XXXX.bin   # Aufzeichnung vom Pi abspielen

Tipp für die beste Karte: live mit --record aufnehmen, danach mit --replay
nochmal in Ruhe auswerten (gleiches Ergebnis, aber ohne Zeitdruck).

Tasten im Fenster:
    s = Karte speichern    r = Karte zurücksetzen    q/ESC = Ende (speichert auch)
"""
import argparse
import math
import pickle
import queue
import struct
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree

import protocol

KARTEN_DIR = Path(__file__).resolve().parent / "karten"   # Ausgaben: karten/<datum_zeit>/

# ======================================================================
# EINSTELLUNGEN
# ======================================================================
MAP_SIZE_M = 60.0        # Kartengröße (quadratisch), Start in der Mitte
RESOLUTION = 0.05        # m pro Zelle
MIN_RANGE = 0.12         # m, kürzere Messungen ignorieren (z. B. eigene Hand)
MIN_SCAN_POINTS = 200    # unvollständige Scans (z. B. der allererste) nicht in die Startkarte
MAX_RANGE = 10.0         # m, T-mini Plus schafft ~12 m
FLIP = False             # True, falls die Karte spiegelverkehrt ist (LiDAR kopfüber)
DESKEW = True            # Bewegungsverzerrung innerhalb eines Scans korrigieren

# Lage vom Flugcontroller (nur wenn der Pi mit --mav läuft)
TILT_COMP = False         # geneigte Scans in die Waagerechte umrechnen
MAX_TILT_DEG = 15.0      # stärker geneigte Scans verwerfen (Ausgleich rechnet bis dahin sauber)
LIDAR_YAW_OFFSET_DEG = 0.0   # Winkel zwischen 0°-Markierung des LiDARs und Drohnen-Vorderseite
LIDAR_ABOVE_RANGEFINDER = 0.0  # m, wie viel höher der LiDAR als der Höhenmesser sitzt
FLOOR_CLEARANCE = 0.10   # m, geneigter Strahl trifft den Boden -> Punkte darunter verwerfen

# Korrelationssuche (normal / nach Verlust)
SEARCH_YAW_DEG = (20.0, 60.0)
SEARCH_XY_M = (0.35, 1.0)
SEARCH_YAW_STEP_DEG = 1.0
FIELD_SIGMA = 0.10       # m, Unschärfe des Wahrscheinlichkeitsfelds

# ICP-Feinabgleich
ICP_MAX_DIST = (0.30, 0.15, 0.08)
ICP_ITER = 25
HUBER = 0.03             # m
MIN_INLIER_RATIO = 0.60  # Anteil Punkte < 10 cm an einer Wand, sonst Scan verwerfen.
                         # Gezählt werden nur Punkte in schon bekannten Bereichen, damit neue
                         # Räume / Fenster / andere Höhen den Scan nicht unnötig verwerfen.
MIN_KNOWN_POINTS = 80    # so viele Punkte müssen in bekannten Bereichen liegen
RETRY_BELOW = 0.80       # Match darunter -> zusätzlich weiträumig suchen, das Bessere nehmen
MIN_MAP_POINTS = 150     # erst ab so vielen Kartenpunkten wird gematcht
START_THRESH = 0.5       # in der Startphase zählt schon eine einmal gesehene Wandzelle
RELOC_AFTER = 12         # Scans (~2 s) ohne Treffer -> Anforderung an den Treffer senken ...
RELAXED_RATIO = 0.40     # ... auf diesen Anteil, damit es nie dauerhaft hängen bleibt

# Occupancy Grid (Log-Odds)
L_OCC, L_FREE = 0.85, -0.4
L_MIN, L_MAX = -4.0, 4.0
L_OCC_THRESH = 1.0       # ab hier gilt eine Zelle als Wand (für das Matching)

# Spiegelfilter: Punkte HINTER einer sicher bekannten Wand, die - an dieser Wand
# zurückgespiegelt - auf einer bekannten Wand landen, sind Spiegelbilder.
MIRROR_FILTER = False    # experimentell, siehe Analyse - standardmäßig aus
L_WALL = 3.5             # so sicher muss die "Wand" sein (L_MAX = 4.0)
MIRROR_BEHIND_MIN = 0.25 # m, Punkt muss mindestens so weit hinter der Wand liegen
MIRROR_CONFIRM = 0.12    # m, zurückgespiegelter Punkt muss so nah an einer Wand landen
MIRROR_MIN_INC = 15.0    # Grad, streifende Strahlen (fast parallel zur Wand) ignorieren
MIRROR_WALL_RADIUS = 0.30   # m, Umgebung zur Prüfung "gerade Fläche"
MIRROR_WALL_MIN_LEN = 0.40  # m, Mindestlänge einer Spiegel-/Wandfläche
MIRROR_CHECK_REFLECTION = False  # Gegenprobe (funktioniert bei geneigtem Spiegel nicht)
MIRROR_CACHE_EVERY = 5      # Wandanalyse alle N Scans aktualisieren
MIRROR_MIN_RUN = 6       # so viele benachbarte Strahlen muss ein Spiegelbild umfassen
WINDOW_PX = 800
DRAW_HZ = 5.0
# ======================================================================


# ----------------------------------------------------------------------
# LiDAR-Treiber (YDLIDAR-Protokoll, 230400 Baud)
# ----------------------------------------------------------------------
PKT_HDR = struct.Struct("<HBBHHH")  # PH, CT, LSN, FSA, LSA, CS


class TMiniPlus:
    def __init__(self, port, baud=230400):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.buf = bytearray()
        self.sample_size = None   # 3 (mit Intensität) oder 2, automatisch erkannt
        self.check_cs = True
        self._tried = 0
        self.cs_errors = 0
        self._a, self._d, self._i = [], [], []

    def start(self):
        self.ser.dtr = True
        self.ser.write(b"\xA5\x65")          # Stop
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.write(b"\xA5\x60")          # Scan starten

    def stop(self):
        try:
            self.ser.write(b"\xA5\x65")
            time.sleep(0.05)
        finally:
            self.ser.close()

    @staticmethod
    def _checksum(ph, ct, lsn, fsa, lsa, data, ss):
        cs = ph ^ (ct | (lsn << 8)) ^ fsa ^ lsa
        if ss == 3:
            for i in range(0, lsn * 3, 3):
                cs ^= data[i]
                cs ^= data[i + 1] | (data[i + 2] << 8)
        else:
            for i in range(0, lsn * 2, 2):
                cs ^= data[i] | (data[i + 1] << 8)
        return cs

    def scans(self):
        """Generator: (winkel_grad, distanz_mm, intensität) je Umdrehung.

        Winkel im Uhrzeigersinn. Intensität 0..255 (0, falls der LiDAR keine liefert).
        """
        while True:
            chunk = self.ser.read(4096)
            if chunk:
                self.buf += chunk
            yield from self._parse()

    def _parse(self):
        buf = self.buf
        while True:
            i = buf.find(b"\xAA\x55")
            if i < 0:
                del buf[:-1]
                return
            del buf[:i]
            if len(buf) < PKT_HDR.size:
                return
            ph, ct, lsn, fsa, lsa, cs = PKT_HDR.unpack_from(buf)
            if lsn == 0 or not (fsa & 1) or not (lsa & 1):
                del buf[:2]
                continue

            if self.sample_size is None:          # Format über Prüfsumme erkennen
                found = False
                for ss in (3, 2):
                    end = PKT_HDR.size + lsn * ss
                    if end > len(buf):
                        return
                    if self._checksum(ph, ct, lsn, fsa, lsa, buf[PKT_HDR.size:end], ss) == cs:
                        self.sample_size, found = ss, True
                        print(f"[lidar] Datenformat erkannt: {ss} Byte pro Messpunkt")
                        break
                if not found:
                    self._tried += 1
                    if self._tried > 100:
                        print("[lidar] WARNUNG: Prüfsumme passt nie - lese ohne Prüfung")
                        self.sample_size, self.check_cs = 3, False
                    else:
                        del buf[:2]
                        continue

            ss = self.sample_size
            total = PKT_HDR.size + lsn * ss
            if len(buf) < total:
                return
            data = bytes(buf[PKT_HDR.size:total])
            if self.check_cs and self._checksum(ph, ct, lsn, fsa, lsa, data, ss) != cs:
                self.cs_errors += 1
                del buf[:2]
                continue
            del buf[:total]

            if ct & 0x01:                          # neue Umdrehung beginnt
                if len(self._a) > 30:
                    yield (np.array(self._a), np.array(self._d, dtype=np.float64),
                           np.array(self._i, dtype=np.uint8))
                self._a, self._d, self._i = [], [], []

            a0 = (fsa >> 1) / 64.0
            diff = (lsa >> 1) / 64.0 - a0
            if diff < 0:
                diff += 360.0
            raw = np.frombuffer(data, dtype=np.uint8).reshape(lsn, ss)
            dist = ((raw[:, -1].astype(np.uint32) << 8) | raw[:, -2]) >> 2
            ang = a0 + (diff * np.arange(lsn) / (lsn - 1) if lsn > 1 else np.zeros(1))
            inten = raw[:, 0] if ss == 3 else np.zeros(lsn, dtype=np.uint8)
            ok = dist > 0
            self._a.extend((ang[ok] % 360.0).tolist())
            self._d.extend(dist[ok].tolist())
            self._i.extend(inten[ok].tolist())


def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:
        text = f"{p.description} {p.manufacturer} {p.hwid}".lower()
        if any(k in text for k in ("cp210", "silicon labs", "ch340", "ch910", "usb-serial", "uart")):
            return p.device
    if len(ports) == 1:
        return ports[0].device
    print("Gefundene Ports:")
    for p in ports:
        print(f"  {p.device}: {p.description}")
    sys.exit("LiDAR-Port nicht eindeutig - bitte mit --port COMx angeben.")


def udp_source(port, stop, stats, pi=None):
    """Empfängt Scans vom Pi. Generator: (t, winkel, distanz, intensität, sensoren).

    sensoren: dict mit roll, pitch, yaw, range, ekf_z zur Scanmitte (None = nicht vorhanden).

    pi gesetzt: Laptop meldet sich jede Sekunde beim Pi (HELLO), der Pi antwortet mit den Scans.
    pi leer:    Laptop wartet auf Port `port`, der Pi sendet von sich aus (--dest).
    """
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("0.0.0.0", 0 if pi else port))
    sock.settimeout(0.2)
    t_hello = 0.0
    if pi:
        print(f"[udp] melde mich beim Pi {pi}:{port} an ...")
    else:
        print(f"[udp] warte auf Daten vom Pi auf Port {port} ...")
    asm = protocol.ScanAssembler()
    sens = protocol.SensorState()
    first = True
    try:
        while not stop.is_set():
            if pi and time.time() - t_hello > 1.0:
                try:
                    sock.sendto(protocol.HELLO, (pi, port))
                except OSError as e:
                    print(f"[udp] Pi nicht erreichbar: {e}")
                t_hello = time.time()
            try:
                msg, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except ConnectionResetError:     # Windows: Pi-Port (noch) nicht offen
                continue
            dec = protocol.decode(msg)
            if dec is None:
                continue
            if first:
                print(f"[udp] Daten von {addr[0]}")
                first = False
            typ, t, data = dec
            if typ != protocol.T_SCAN:
                sens.add(typ, t, data)
                stats["pose"] = data if typ == protocol.T_POSE else stats.get("pose")
                continue
            scan = asm.add(t, data)
            stats["lost"] = asm.lost
            if scan:
                t, A, D, I = scan
                yield (t, np.array(A), np.array(D, dtype=np.float64), np.array(I, dtype=np.uint8),
                       sens.at(t - SCAN_MID))
    finally:
        sock.close()


def bin_source(path):
    """Liest eine Pi-Aufzeichnung (.bin). Generator wie udp_source."""
    asm = protocol.ScanAssembler()
    sens = protocol.SensorState()
    pending = []                                   # Scans kurz zurückhalten, bis Lage-Daten da sind
    for msg in protocol.read_log(path):
        dec = protocol.decode(msg)
        if dec is None:
            continue
        typ, t, data = dec
        if typ != protocol.T_SCAN:
            sens.add(typ, t, data)
            continue
        scan = asm.add(t, data)
        if scan:
            pending.append(scan)
            while pending and pending[0][0] < t - 0.3:
                yield _with_sensors(pending.pop(0), sens)
    for sc in pending:
        yield _with_sensors(sc, sens)


def _with_sensors(scan, sens):
    t, A, D, I = scan
    return t, np.array(A), np.array(D, dtype=np.float64), np.array(I, dtype=np.uint8), sens.at(t - SCAN_MID)


SCAN_MID = 0.08   # s, Zeitstempel = Scan-Ende; Lage zur Scanmitte nehmen


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


def pkl_source(path):
    """Liest eine Laptop-Aufzeichnung (.pkl). Generator: (t, winkel, distanz, intensität, höhe|None)."""
    with open(path, "rb") as f:
        data = pickle.load(f)
    for item in data:
        inten = item[3] if len(item) > 3 else np.zeros(len(item[1]), np.uint8)
        yield item[0], item[1], item[2], inten, (item[4] if len(item) > 4 else None)


def replay_source(path):
    """Wählt das Format anhand der Dateiendung."""
    return bin_source(path) if path.lower().endswith(".bin") else pkl_source(path)


def udp_thread(port, q, stop, record, stats, pi=None):
    for t, ang, dist, inten, sens in udp_source(port, stop, stats, pi):
        if record is not None:
            record.append((t, ang, dist, inten))
        q.put((ang, dist, sens))


def lidar_thread(port, q, stop, record):
    lidar = TMiniPlus(port)
    lidar.start()
    print(f"[lidar] läuft auf {port}")
    try:
        for ang, dist, inten in lidar.scans():
            if stop.is_set():
                break
            if record is not None:
                record.append((time.time(), ang, dist, inten))
            q.put((ang, dist))                 # nichts verwerfen: jeder Scan zählt
    finally:
        lidar.stop()


def replay_thread(path, q, stop, speed):
    print(f"[replay] {path}")
    t_prev = None
    n = 0
    for t, ang, dist, inten, extra in replay_source(path):
        if stop.is_set():
            return
        if t_prev is not None and speed > 0:
            time.sleep(max(0.0, (t - t_prev) / speed))
        t_prev = t
        q.put((ang, dist, extra if isinstance(extra, dict) else None))
        n += 1
    print(f"[replay] {n} Scans abgespielt")
    q.put(None)


# ----------------------------------------------------------------------
# Geometrie
# ----------------------------------------------------------------------
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


# ----------------------------------------------------------------------
# Matching
# ----------------------------------------------------------------------
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


# ----------------------------------------------------------------------
# Karte
# ----------------------------------------------------------------------
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


def draw(grid, pose, traj, scan_world, info):
    img = cv2.cvtColor(grid.image(), cv2.COLOR_GRAY2BGR)
    n = grid.n

    def px(xy):
        c = grid.to_cell(np.atleast_2d(xy))
        return np.column_stack((c[:, 0], n - 1 - c[:, 1]))

    if len(scan_world):
        q = px(scan_world)
        ok = (q[:, 0] >= 0) & (q[:, 0] < n) & (q[:, 1] >= 0) & (q[:, 1] < n)
        img[q[ok, 1], q[ok, 0]] = (0, 200, 0)
    if len(traj) > 1:
        cv2.polylines(img, [px(np.array(traj)[:, :2]).astype(np.int32)], False, (0, 0, 255), 1)
    p = px(pose[:2])[0]
    tip = px(np.array(pose[:2]) + 0.5 * np.array([math.cos(pose[2]), math.sin(pose[2])]))[0]
    cv2.arrowedLine(img, tuple(int(v) for v in p), tuple(int(v) for v in tip), (255, 0, 0), 2, tipLength=0.4)

    half = int(min(n, max(200, 2 * MAX_RANGE / grid.res)) / 2)
    cx = int(np.clip(p[0], half, n - half))
    cy = int(np.clip(p[1], half, n - half))
    view = img[cy - half:cy + half, cx - half:cx + half]
    view = cv2.resize(view, (WINDOW_PX, WINDOW_PX), interpolation=cv2.INTER_NEAREST)
    cv2.putText(view, info, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return view


# ----------------------------------------------------------------------
class Mapper:
    def __init__(self):
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="z. B. COM5 (Windows) oder /dev/ttyUSB0 (Linux)")
    ap.add_argument("--record", help="Rohdaten in diese Datei aufzeichnen (.pkl)")
    ap.add_argument("--replay", help="Aufzeichnung abspielen statt LiDAR (.pkl vom Laptop oder .bin vom Pi)")
    ap.add_argument("--pi", metavar="IP", help="Scans vom Pi holen (Tailscale-IP des Pi, dort läuft pi_streamer.py)")
    ap.add_argument("--udp", type=int, metavar="PORT", help="Port (Standard 5005); ohne --pi: warten, bis der Pi sendet")
    ap.add_argument("--speed", type=float, default=1.0, help="Replay-Geschwindigkeit (0 = so schnell wie möglich)")
    ap.add_argument("--save", help="Dateiname ohne Endung (Standard: karten/<datum_zeit>/karte)")
    ap.add_argument("--no-gui", action="store_true")
    args = ap.parse_args()
    args.save, args.record = output_paths(args.save, args.record, "karte")

    q = queue.Queue(maxsize=50 if args.replay else 0)
    stop = threading.Event()
    record = [] if args.record else None
    stats = {"lost": 0, "pose": None}
    if args.replay:
        th = threading.Thread(target=replay_thread, args=(args.replay, q, stop, args.speed), daemon=True)
    elif args.pi or args.udp:
        args.udp = args.udp or protocol.DEFAULT_PORT
        th = threading.Thread(target=udp_thread, args=(args.udp, q, stop, record, stats, args.pi), daemon=True)
    else:
        port = args.port or find_port()
        th = threading.Thread(target=lidar_thread, args=(port, q, stop, record), daemon=True)
    th.start()

    mp = Mapper()
    t_draw, t_last, hz = 0.0, time.time(), 0.0
    print("Läuft. Im Fenster: s = speichern, r = zurücksetzen, q = Ende")

    try:
        while True:
            try:
                item = q.get(timeout=3)
            except queue.Empty:
                if not th.is_alive():
                    print("LiDAR-Thread beendet.")
                    break
                print("Warte auf Scans ... " + ("(läuft pi_streamer.py auf dem Pi? richtige IP?)" if args.udp
                                                else "(LiDAR angeschlossen und dreht sich?)"))
                continue
            if item is None:
                break
            world = mp.process(*item)

            now = time.time()
            hz = 0.9 * hz + 0.1 / max(now - t_last, 1e-3)
            t_last = now
            if args.no_gui or now - t_draw < 1.0 / DRAW_HZ:
                continue
            t_draw = now
            backlog = q.qsize() if not args.replay else 0
            state = "VERLOREN - langsam zurück zu bekanntem Bereich" if mp.lost > 3 else ""
            tilt = f"  Neigung={mp.last_tilt:.0f}deg (verw. {mp.n_tilt})" if mp.last_tilt is not None else ""
            info = (f"match={mp.last_ratio:.0%}  {hz:.1f} Hz  verworfen={mp.n_rejected}{tilt}"
                    + (f"  Rueckstand={backlog}" if backlog > 2 else "")
                + (f"  UDP-Verlust={stats['lost']}" if args.udp else "") + f"  {state}")
            cv2.imshow("LiDAR-Karte", draw(mp.grid, mp.pose, mp.traj, world, info))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                mp.grid.save(args.save, mp.traj)
            if key == ord("r"):
                mp = Mapper()
                print("Karte zurückgesetzt")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print(f"{mp.n_scans} Scans verarbeitet, {mp.n_rejected} verworfen, {mp.n_tilt} wegen Neigung verworfen")
        mp.grid.save(args.save, mp.traj)
        if record:
            with open(args.record, "wb") as f:
                pickle.dump(record, f)
            print(f"Rohdaten: {args.record} ({len(record)} Scans)")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
