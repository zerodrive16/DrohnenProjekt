"""Datenquellen: LiDAR per USB, UDP vom Pi (pi_streamer.py), Aufzeichnungen (.bin/.pkl).

Alle Quellen liefern Scans als (zeit, winkel_grad, distanz_mm, intensität, extra):
extra = Sensordaten vom FC (dict) bei Pi-Quellen, gespeicherte Höhe bei .pkl, sonst None.
"""
import pickle
import struct
import sys
import time

import numpy as np

import protocol
from config import *  # noqa: F401,F403

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


def usb_source(port, stop):
    """LiDAR direkt am Rechner. Generator wie oben (extra = None)."""
    lidar = TMiniPlus(port)
    lidar.start()
    print(f"[lidar] läuft auf {port}")
    try:
        for ang, dist, inten in lidar.scans():
            if stop.is_set():
                return
            yield time.time(), ang, dist, inten, None
    finally:
        lidar.stop()
