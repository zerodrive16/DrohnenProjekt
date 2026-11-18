"""Nachrichtenformat zwischen Pi (Sender) und Laptop (Empfänger).

Muss auf Pi und Laptop identisch sein. Nur Standardbibliothek (läuft auf dem Pi ohne numpy).

Jede Nachricht = ein UDP-Paket bzw. ein Eintrag in der Log-Datei:

    Header:  "DR" | typ (u8) | zeitstempel (f64, Pi-Zeit in s)
    T_SCAN:  scan_id (u32) | teil (u8) | teile_gesamt (u8) | n (u16)
             + n x winkel (u16, 1/100 Grad, im Uhrzeigersinn)
             + n x distanz (u16, mm)
             + n x intensität (u8)
    T_POSE:  roll, pitch, yaw (f32, rad)  x, y, z (f32, m, NED; NaN = unbekannt)
             (Flugcontroller: ATTITUDE + LOCAL_POSITION_NED)
    T_RANGE: distanz (f32, m) | ausrichtung (u8, MAVLink: 25 = nach unten)
             (Flugcontroller: DISTANCE_SENSOR, z. B. MTF-01P)

Ein Scan wird in Teile zu max. 200 Punkten (~1 KB) zerlegt, damit die Pakete
auch über Tailscale (MTU 1280) nicht fragmentiert werden.
Punkte bleiben in Aufnahmereihenfolge (wichtig für die Entzerrung).

Log-Datei (.bin): Folge von [länge (u16) | nachricht].

Verbindungsaufbau: Der Laptop schickt jede Sekunde HELLO an den Pi (Port 5005).
Der Pi sendet die Scans an die Adresse, von der das letzte HELLO kam. So klappt
es auch, wenn der Pi selbst keine Verbindungen aufbauen darf (Tailscale-Sharing).
"""
import struct

MAGIC = b"DR"
T_SCAN = 1
T_POSE = 2
T_RANGE = 3

HDR = struct.Struct("<2sBd")
SCAN_HDR = struct.Struct("<IBBH")
POSE = struct.Struct("<6f")
RANGE = struct.Struct("<fB")
LEN = struct.Struct("<H")

MAX_POINTS_PER_CHUNK = 200
HELLO = b"DRHELLO1"
DEFAULT_PORT = 5005


def encode_scan(scan_id, t, angles_deg, dists_mm, intens):
    """Scan -> Liste von Nachrichten (bytes)."""
    n_total = len(angles_deg)
    n_chunks = max(1, -(-n_total // MAX_POINTS_PER_CHUNK))
    msgs = []
    for c in range(n_chunks):
        sl = slice(c * MAX_POINTS_PER_CHUNK, (c + 1) * MAX_POINTS_PER_CHUNK)
        a = [int(round(x * 100)) % 36000 for x in angles_deg[sl]]
        d = [min(int(x), 65535) for x in dists_mm[sl]]
        i = [min(int(x), 255) for x in intens[sl]]
        n = len(a)
        body = struct.pack(f"<{n}H{n}H{n}B", *a, *d, *i)
        msgs.append(HDR.pack(MAGIC, T_SCAN, t) + SCAN_HDR.pack(scan_id & 0xFFFFFFFF, c, n_chunks, n) + body)
    return msgs


def encode_pose(t, roll, pitch, yaw, x, y, z):
    return HDR.pack(MAGIC, T_POSE, t) + POSE.pack(roll, pitch, yaw, x, y, z)


def encode_range(t, dist_m, orientation):
    return HDR.pack(MAGIC, T_RANGE, t) + RANGE.pack(dist_m, orientation)


def decode(msg):
    """-> (typ, t, daten) oder None.

    T_SCAN: daten = (scan_id, teil, teile_gesamt, winkel_liste, distanz_liste, intensität_liste)
    T_POSE: daten = (roll, pitch, yaw, x, y, z)
    T_RANGE: daten = (distanz_m, ausrichtung)
    """
    if len(msg) < HDR.size:
        return None
    magic, typ, t = HDR.unpack_from(msg)
    if magic != MAGIC:
        return None
    off = HDR.size
    if typ == T_SCAN:
        if len(msg) < off + SCAN_HDR.size:
            return None
        sid, part, parts, n = SCAN_HDR.unpack_from(msg, off)
        off += SCAN_HDR.size
        if len(msg) < off + 5 * n:
            return None
        vals = struct.unpack_from(f"<{n}H{n}H{n}B", msg, off)
        return typ, t, (sid, part, parts, [v / 100.0 for v in vals[:n]], list(vals[n:2 * n]), list(vals[2 * n:]))
    if typ == T_POSE and len(msg) >= off + POSE.size:
        return typ, t, POSE.unpack_from(msg, off)
    if typ == T_RANGE and len(msg) >= off + RANGE.size:
        return typ, t, RANGE.unpack_from(msg, off)
    return None


class SensorState:
    """Puffert Lage und Höhenmesser vom Flugcontroller, liefert Werte zu einem Zeitpunkt."""

    def __init__(self):
        self.att = []      # (t, roll, pitch, yaw)
        self.rng = []      # (t, distanz)
        self.ekf_z = None  # (t, z)

    def add(self, typ, t, data):
        if typ == T_POSE:
            self.att.append((t, data[0], data[1], data[2]))
            if data[5] == data[5]:                       # nicht NaN
                self.ekf_z = (t, data[5])
            if len(self.att) > 400:
                del self.att[:200]
        elif typ == T_RANGE and data[1] == 25:            # nur nach unten gerichtet
            self.rng.append((t, data[0]))
            if len(self.rng) > 200:
                del self.rng[:100]

    def at(self, t):
        """-> dict mit roll, pitch, yaw, range, ekf_z (jeweils None, wenn nicht vorhanden)."""
        out = {"roll": None, "pitch": None, "yaw": None, "range": None, "ekf_z": None}
        if self.att:
            best = min(self.att[-60:], key=lambda a: abs(a[0] - t))
            if abs(best[0] - t) < 0.2:
                out["roll"], out["pitch"], out["yaw"] = best[1], best[2], best[3]
        if self.rng:
            best = min(self.rng[-30:], key=lambda r: abs(r[0] - t))
            if abs(best[0] - t) < 0.4:
                out["range"] = best[1]
        if self.ekf_z and abs(self.ekf_z[0] - t) < 0.5:
            out["ekf_z"] = self.ekf_z[1]
        return out


class ScanAssembler:
    """Setzt die Teile eines Scans wieder zusammen. Unvollständige Scans werden verworfen."""

    def __init__(self):
        self.pending = {}          # scan_id -> {teil: (winkel, distanz, intensität)}, t, teile
        self.lost = 0
        self.complete = 0
        self.last_id = None

    def add(self, t, data):
        """-> (t, winkel, distanz, intensität) wenn ein Scan vollständig ist, sonst None."""
        sid, part, parts, a, d, i = data
        entry = self.pending.setdefault(sid, [t, parts, {}])
        entry[2][part] = (a, d, i)
        if len(entry[2]) < parts:
            # ältere, unvollständige Scans aufgeben
            if len(self.pending) > 3:
                for old in sorted(self.pending)[:-3]:
                    del self.pending[old]
                    self.lost += 1
            return None
        del self.pending[sid]
        for old in [k for k in self.pending if k < sid]:   # was davor fehlte, kommt nicht mehr
            del self.pending[old]
            self.lost += 1
        self.complete += 1
        A, D, I = [], [], []
        for p in range(parts):
            a, d, i = entry[2][p]
            A += a; D += d; I += i
        return entry[0], A, D, I


def read_log(path):
    """Generator über alle Nachrichten (bytes) einer Log-Datei."""
    with open(path, "rb") as f:
        while True:
            head = f.read(LEN.size)
            if len(head) < LEN.size:
                return
            (n,) = LEN.unpack(head)
            msg = f.read(n)
            if len(msg) < n:
                return
            yield msg
