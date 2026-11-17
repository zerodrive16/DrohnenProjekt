#!/usr/bin/env python3
"""aufnahme.py - läuft auf dem Pi: LiDAR (YDLIDAR T-mini Plus) und Flugcontroller aufnehmen.

Schreibt alles mit Zeitstempel in eine JSONL-Datei und schickt es auf Wunsch live per UDP an den Laptop.
Das Skript steuert den Flugcontroller NICHT: es liest nur Nachrichten mit (und bittet um Sendeintervalle).

    python aufnahme.py                              # LiDAR + FC aufnehmen
    python aufnahme.py --no-fc                      # nur LiDAR (solange der FC nicht verbunden ist)
    python aufnahme.py --send 192.168.43.10:5005    # zusätzlich live an den Laptop
    python aufnahme.py --duration 60                # nach 60 s selbst beenden
Beenden mit Strg+C.

Voraussetzungen: YDLidar-SDK (Python-Modul 'ydlidar'), pymavlink (nur mit FC), Rechte auf den Ports (Gruppe dialout).
Die SDK-Optionen für das T-mini Plus stammen aus eurem lidar_test.py; ich habe sie nicht am Gerät geprüft.
"""
import argparse
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from datetime import datetime

from aufnahmeformat import pack_scan


class Sink:
    """Schreibt Datensätze in die Datei (und per UDP). Thread-sicher."""

    def __init__(self, path, target=None):
        self.f = open(path, "w", encoding="utf-8")
        self.lock = threading.Lock()
        self.sock, self.target = None, target
        self.counts, self.send_errors = {}, 0
        if target:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)

    def _send(self, data):
        try:
            self.sock.sendto(data, self.target)
        except OSError:
            self.send_errors += 1  # Netz weg oder Puffer voll: die Datei auf dem Pi bleibt vollständig

    def write(self, rec):
        line = json.dumps(rec, separators=(",", ":"))
        with self.lock:
            self.f.write(line + "\n")
            self.f.flush()
            self.counts[rec["type"]] = self.counts.get(rec["type"], 0) + 1
            if self.sock and rec["type"] != "scan":
                self._send(b"J" + line.encode())

    def scan(self, seq, t, pts):
        self.write({"type": "scan", "seq": seq, "t": round(t, 3), "pts": [list(p) for p in pts]})
        if self.sock:
            for d in pack_scan(seq, t, pts):
                self._send(d)

    def close(self):
        with self.lock:
            self.f.close()


def fc_loop(a, sink, stop):
    from pymavlink import mavutil
    mav = mavutil.mavlink_connection(a.fc_port, baud=a.fc_baud)
    t0 = time.time()
    while not stop.is_set():
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is not None and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID \
                and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            mav.target_system, mav.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            break
        if time.time() - t0 > a.fc_wait:
            print("WARNUNG: kein Heartbeat vom FC auf %s - es wird ohne FC aufgenommen." % a.fc_port, flush=True)
            return
    else:
        return
    print("FC verbunden (System %d)." % mav.target_system, flush=True)
    for msgid, hz in ((30, 20), (32, 10), (132, 10)):  # ATTITUDE, LOCAL_POSITION_NED, DISTANCE_SENSOR
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, int(1e6 / hz), 0, 0, 0, 0, 0)
    while not stop.is_set():
        m = mav.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"], blocking=True, timeout=1)
        if m is None:
            continue
        t, ty = round(time.time(), 3), m.get_type()
        if ty == "ATTITUDE":
            sink.write({"type": "att", "t": t, "tb": m.time_boot_ms, "roll": round(m.roll, 4),
                        "pitch": round(m.pitch, 4), "yaw": round(m.yaw, 4)})
        elif ty == "LOCAL_POSITION_NED":
            sink.write({"type": "pos", "t": t, "tb": m.time_boot_ms, "x": round(m.x, 3), "y": round(m.y, 3),
                        "z": round(m.z, 3)})
        else:
            sink.write({"type": "rng", "t": t, "tb": m.time_boot_ms, "cm": m.current_distance,
                        "orient": m.orientation})


def lidar_loop(a, sink, stop):
    try:
        import ydlidar
    except ImportError:
        sys.exit("Modul 'ydlidar' fehlt: YDLidar-SDK bauen und im venv installieren (siehe SDK-README).")
    if not os.path.exists(a.lidar_port):
        sys.exit("%s nicht gefunden. 'ls /dev/ttyUSB*' prüfen." % a.lidar_port)
    if not os.access(a.lidar_port, os.R_OK | os.W_OK):
        sys.exit("Keine Rechte auf %s. 'sudo usermod -aG dialout $USER', danach neu einloggen." % a.lidar_port)

    ydlidar.os_init()
    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, a.lidar_port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, a.lidar_baud)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, a.scan_hz)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, a.sample_rate)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 12.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.05)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)  # so heißt die Option im SDK (Tippfehler dort)
    if not laser.initialize():
        sys.exit("LiDAR-Initialisierung fehlgeschlagen: %s" % laser.DescribeError())
    if not laser.turnOn():
        laser.disconnecting()
        sys.exit("LiDAR-Start fehlgeschlagen: %s" % laser.DescribeError())
    print("LiDAR läuft auf %s @ %d Baud." % (a.lidar_port, a.lidar_baud), flush=True)

    scan = ydlidar.LaserScan()
    seq, t_start, t_status, n_status = 0, time.time(), time.time(), 0
    try:
        while ydlidar.os_isOk() and not stop.is_set():
            if a.duration and time.time() - t_start >= a.duration:
                break
            if not laser.doProcessSimple(scan):
                time.sleep(0.05)
                continue
            t = time.time()
            pts = []
            for p in scan.points:
                if p.range > 0:
                    pts.append((int(round(math.degrees(p.angle) * 100)) % 36000, min(int(round(p.range * 1000)), 65535)))
            if not pts:
                continue
            sink.scan(seq, t, pts)
            seq += 1
            n_status += 1
            if t - t_status >= 2.0:
                c = sink.counts
                print("%5.0f s | Scans %.1f/s (zuletzt %d Punkte) | FC: att %d, pos %d, rng %d%s" % (
                    t - t_start, n_status / (t - t_status), len(pts), c.get("att", 0), c.get("pos", 0), c.get("rng", 0),
                    " | Sendefehler %d" % sink.send_errors if sink.send_errors else ""), flush=True)
                t_status, n_status = t, 0
    finally:
        laser.turnOff()
        laser.disconnecting()
        print("LiDAR gestoppt.", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="aufnahmen", help="Ordner für die Aufnahmedateien")
    ap.add_argument("--lidar-port", default="/dev/ttyUSB0")
    ap.add_argument("--lidar-baud", type=int, default=230400)
    ap.add_argument("--scan-hz", type=float, default=10.0, help="Drehfrequenz (T-mini Plus: 6-12 Hz)")
    ap.add_argument("--sample-rate", type=int, default=4, help="SDK-Abtastrate (4 = 4 kHz laut euren Skript)")
    ap.add_argument("--fc-port", default="/dev/serial0")
    ap.add_argument("--fc-baud", type=int, default=115200)
    ap.add_argument("--fc-wait", type=float, default=15.0, help="Sekunden auf FC-Heartbeat warten")
    ap.add_argument("--no-fc", action="store_true", help="nur das LiDAR aufnehmen")
    ap.add_argument("--send", metavar="HOST:PORT", help="live per UDP an den Laptop schicken")
    ap.add_argument("--duration", type=float, default=0, help="nach so vielen Sekunden beenden (0 = bis Strg+C)")
    a = ap.parse_args()

    target = None
    if a.send:
        host, _, port = a.send.rpartition(":")
        if not host or not port.isdigit():
            sys.exit("--send erwartet HOST:PORT, z. B. 192.168.43.10:5005")
        target = (host, int(port))
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "aufnahme_%s.jsonl" % datetime.now().strftime("%Y%m%d_%H%M%S"))
    sink = Sink(path, target)
    sink.write({"type": "meta", "format": 1, "start": datetime.now().isoformat(timespec="seconds"),
                "lidar": {"port": a.lidar_port, "baud": a.lidar_baud, "scan_hz": a.scan_hz, "sample_rate": a.sample_rate},
                "fc": None if a.no_fc else {"port": a.fc_port, "baud": a.fc_baud},
                "einheiten": {"pts": "[Winkel in 0,01 Grad, Abstand in mm]", "att": "rad", "pos": "m, z nach unten"},
                "hinweis": "Winkel unverändert vom YDLidar-SDK; Einbaurichtung und Vorzeichen müssen noch geprüft werden."})
    print("Aufnahme nach %s" % path, flush=True)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    if not a.no_fc:
        threading.Thread(target=fc_loop, args=(a, sink, stop), daemon=True).start()
    try:
        lidar_loop(a, sink, stop)
    finally:
        stop.set()
        sink.close()
        print("Fertig: %s" % path, flush=True)


if __name__ == "__main__":
    main()
