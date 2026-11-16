"""Gemeinsames Format für Aufnahme (Pi) und Empfänger (Laptop). Nur Standardbibliothek.

Aufnahmedatei: JSON Lines (eine Zeile = ein Datensatz). Datensätze:
  {"type":"meta", ...}                                   Einstellungen der Aufnahme
  {"type":"scan","seq":n,"t":s,"pts":[[cdeg,mm],...]}    ein LiDAR-Umlauf
        cdeg = Winkel in 0,01 Grad (0..35999, wie vom SDK geliefert), mm = Abstand in Millimetern
  {"type":"att","t":s,"tb":ms,"roll":r,"pitch":p,"yaw":y}   Lage vom FC in Radiant
  {"type":"pos","t":s,"tb":ms,"x":m,"y":m,"z":m}            LOCAL_POSITION_NED (z nach unten!)
  {"type":"rng","t":s,"tb":ms,"cm":c,"orient":o}            Abstandssensor (Höhe), cm
t = Uhrzeit des Pi in Sekunden (time.time()). Alle Datensätze eines Pi haben dieselbe Uhr; der Pi hat
keine batteriegepufferte Uhr, die absolute Zeit kann also falsch sein, die Abstände stimmen.

Netzwerk (UDP): Scans werden in Stücke zu höchstens CHUNK_PTS Punkten geteilt, damit jedes Paket in
einen Netzwerk-Frame passt (keine IP-Fragmentierung). Alle anderen Datensätze gehen als b"J" + JSON.
"""

import json
import struct

HEAD = struct.Struct("<2sIdBBH")  # Kennung, seq, t, Stück, Anzahl Stücke, Punkte im Stück
MAGIC_SCAN = b"YS"
CHUNK_PTS = 250


def pack_scan(seq, t, pts):
    """Teilt einen Scan in Datagramme (<= 18 + 4*CHUNK_PTS Byte)."""
    n = len(pts)
    nch = max(1, (n + CHUNK_PTS - 1) // CHUNK_PTS)
    out = []
    for i in range(nch):
        part = pts[i * CHUNK_PTS:(i + 1) * CHUNK_PTS]
        body = b"".join(struct.pack("<HH", a, r) for a, r in part)
        out.append(HEAD.pack(MAGIC_SCAN, seq, t, i, nch, len(part)) + body)
    return out


class Reassembler:
    """Setzt Scan-Stücke wieder zusammen. Unvollständige Scans verfallen nach `timeout` Sekunden."""

    def __init__(self, timeout=1.0):
        self.timeout = timeout
        self.parts = {}
        self.dropped = 0
        self.complete = 0

    def add(self, data, now):
        """Gibt (seq, t, pts) zurück, wenn der Scan vollständig ist, sonst None."""
        if len(data) < HEAD.size:
            return None
        magic, seq, t, i, nch, n = HEAD.unpack_from(data)
        if magic != MAGIC_SCAN or len(data) != HEAD.size + 4 * n or i >= nch:
            return None
        pts = [struct.unpack_from("<HH", data, HEAD.size + 4 * k) for k in range(n)]
        e = self.parts.setdefault(seq, {"t": t, "nch": nch, "chunks": {}, "first": now})
        e["chunks"][i] = pts
        result = None
        if len(e["chunks"]) == e["nch"]:
            allpts = []
            for k in range(e["nch"]):
                allpts.extend(e["chunks"][k])
            del self.parts[seq]
            self.complete += 1
            result = (seq, t, [list(p) for p in allpts])
        self.expire(now)
        return result

    def expire(self, now):
        """Verwirft Scans, deren Stücke zu lange unvollständig sind (Paketverlust)."""
        for s in [s for s, v in self.parts.items() if now - v["first"] > self.timeout]:
            del self.parts[s]
            self.dropped += 1


def read_jsonl(path):
    """Liest eine Aufnahmedatei. Eine halb geschriebene letzte Zeile (Absturz, Stromverlust) wird übersprungen."""
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except ValueError:
                continue
