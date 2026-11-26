"""Läuft auf dem Raspberry Pi: LiDAR lesen -> per UDP an den Laptop + lokal aufzeichnen.

Dateien auf dem Pi (alle im selben Ordner):
    pi_streamer.py, tmini_plus.py, protocol.py
Installation auf dem Pi:
    sudo apt install python3-serial
    Mit Flugcontroller zusätzlich pymavlink (im venv):
        python3 -m venv --system-site-packages ~/venv && ~/venv/bin/pip install pymavlink
        und dann mit ~/venv/bin/python3 starten

Start (normal): Pi wartet, bis sich der Laptop meldet
    python3 pi_streamer.py
    Laptop:  python main.py --pi PI_IP          (PI_IP = Tailscale-IP des Pi)

    ~/venv/bin/python3 pi_streamer.py --mav /dev/serial0
                                                    # + Lage/Höhe vom Flugcontroller
Hindernisvermeidung: Mit --mav schickt der Pi aus jedem LiDAR-Scan die kürzesten
Abstände in 72 Richtungen (je 5°) als MAVLink OBSTACLE_DISTANCE an den Flugcontroller.
ArduPilot bremst damit vor Hindernissen (Parameter: PRX1_TYPE=2, AVOID_ENABLE mit
Proximity-Bit, siehe LIESMICH.md). Abschalten mit --no-obstacle.

Alternativ (nur wenn der Pi den Laptop selbst erreichen darf):
    python3 pi_streamer.py --dest LAPTOP_IP:5005    Laptop: python main.py --udp 5005

Aufzeichnungen landen in logs/scan_JJJJMMTT_HHMMSS.bin. Am Laptop abspielen mit:
    python main.py --replay scan_....bin
"""
import argparse
import math
import os
import socket
import threading
import time

from protocol import DEFAULT_PORT, HELLO, LEN, encode_pose, encode_range, encode_scan
from tmini_plus import TMiniPlus


N_SECTORS = 72                 # 72 x 5 Grad = 360 Grad
UNKNOWN = 65535                # MAVLink: keine Messung in diesem Sektor


def obstacle_sectors(angles_deg, dists_mm, yaw_offset_deg, min_m, max_m):
    """Kürzester Abstand (cm) je 5°-Sektor, Sektor 0 = vorne, im Uhrzeigersinn.

    Der LiDAR misst wie MAVLink im Uhrzeigersinn. yaw_offset = Winkel, um den die
    0°-Markierung des LiDARs gegenüber der Drohnenvorderseite verdreht ist.
    Messungen unter min_m (eigene Arme/Propeller) werden ignoriert.
    """
    width = 360.0 / N_SECTORS
    best = [UNKNOWN] * N_SECTORS
    lo, hi = int(min_m * 100), int(max_m * 100)
    for a, d in zip(angles_deg, dists_mm):
        cm = int(d / 10)
        if cm < lo:
            continue
        k = int(((a + yaw_offset_deg + width / 2) % 360.0) // width)
        v = min(cm, hi + 1)                  # weiter als max = frei
        if v < best[k]:
            best[k] = v
    return best


class FcLink:
    """Gemeinsame MAVLink-Verbindung: der MAVLink-Thread öffnet sie, der LiDAR-Thread sendet mit."""

    def __init__(self):
        self.m = None
        self.lock = threading.Lock()
        self.sectors = None                  # letzte Hindernisdaten aus dem LiDAR
        self.sectors_t = 0.0


class Output:
    """Schickt Nachrichten per UDP an den Laptop und schreibt sie in die Log-Datei.

    Modus "listen": Ziel ist der Laptop, der zuletzt HELLO geschickt hat.
    Modus "dest":   festes Ziel.
    """

    def __init__(self, dest, listen_port, log_path):
        self.lock = threading.Lock()
        self.dest, self.last_hello = None, 0.0
        self.fixed = dest is not None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.fixed:
            host, port = dest.rsplit(":", 1)
            self.dest = (host, int(port))
        else:
            self.sock.bind(("0.0.0.0", listen_port))
            threading.Thread(target=self._hello_loop, daemon=True).start()
        self.log = open(log_path, "wb") if log_path else None
        self.send_errors = 0
        self.connected = self.fixed

    def _hello_loop(self):
        while True:
            try:
                msg, addr = self.sock.recvfrom(64)
            except OSError:
                return
            if msg.startswith(HELLO):
                if self.dest != addr or not self.connected:
                    print(f"[udp] Laptop verbunden: {addr[0]}:{addr[1]}")
                self.dest, self.last_hello, self.connected = addr, time.time(), True

    def put(self, msg):
        with self.lock:
            if self.log:
                self.log.write(LEN.pack(len(msg)) + msg)
            if self.dest is None:
                return
            if not self.fixed and time.time() - self.last_hello > 5.0:
                if self.connected:
                    print("[udp] Laptop meldet sich nicht mehr - senden pausiert (Aufzeichnung läuft weiter)")
                    self.connected = False
                return
            try:
                self.sock.sendto(msg, self.dest)
            except OSError:
                self.send_errors += 1     # z. B. WLAN kurz weg - Aufzeichnung läuft weiter

    def flush(self):
        with self.lock:
            if self.log:
                self.log.flush()

    def close(self):
        with self.lock:
            if self.log:
                self.log.close()
                self.log = None


def lidar_loop(args, out, stats, stop, fc):
    lidar = TMiniPlus(args.lidar)
    lidar.start()
    print(f"[lidar] gestartet auf {args.lidar}")
    scan_id = 0
    try:
        for t, angles, dists, intens in lidar.read_scans():
            if stop.is_set():
                break
            for msg in encode_scan(scan_id, t, angles, dists, intens):
                out.put(msg)
            if not args.no_obstacle:
                sec = obstacle_sectors(angles, dists, args.lidar_yaw_offset, args.obstacle_min, args.obstacle_max)
                fc.sectors, fc.sectors_t = sec, time.time()
                near = [v for v in sec if v != UNKNOWN]
                stats["obst_min"] = min(near) / 100.0 if near else None
            scan_id += 1
            stats["scans"] += 1
            stats["points"] += len(angles)
            stats["cs_err"] = lidar.cs_errors
            stats["format"] = lidar.sample_size
    finally:
        lidar.stop()


def obstacle_loop(args, stats, stop, fc):
    """Schickt die letzten Hindernisdaten mit 10 Hz an den FC (ArduPilot erwartet >= 10 Hz,
    der LiDAR liefert ~6 Scans/s). Sind die Daten älter als 0,5 s (LiDAR ausgefallen),
    wird nichts mehr gesendet - der FC merkt dann, dass der Sensor fehlt."""
    while not stop.is_set():
        time.sleep(0.1)
        if fc.m is None or fc.sectors is None or time.time() - fc.sectors_t > 0.5:
            continue
        try:
            with fc.lock:
                fc.m.mav.obstacle_distance_send(
                    int(time.time() * 1e6), 0, fc.sectors, 0,
                    int(args.obstacle_min * 100), int(args.obstacle_max * 100),
                    increment_f=360.0 / N_SECTORS, angle_offset=0.0, frame=12)   # 12 = MAV_FRAME_BODY_FRD
            stats["obst"] += 1
        except Exception as e:                         # FC-Verbindung weg: LiDAR läuft weiter
            stats["obst_err"] = str(e)


def mav_loop(args, out, stats, stop, fc):
    from pymavlink import mavutil

    print(f"[mav] verbinde mit {args.mav} ...")
    m = mavutil.mavlink_connection(args.mav, baud=args.mav_baud,
                                   source_system=1, source_component=191)  # 191 = Begleitrechner
    if m.wait_heartbeat(timeout=15) is None:
        print("[mav] kein Heartbeat vom Flugcontroller - nur LiDAR wird gesendet")
        return
    print(f"[mav] Heartbeat von System {m.target_system}")
    fc.m = m
    for msg_id in (30, 32, 132):  # ATTITUDE, LOCAL_POSITION_NED, DISTANCE_SENSOR
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, int(1e6 / args.mav_rate), 0, 0, 0, 0, 0)
    pos, pos_t = (math.nan,) * 3, 0.0
    while not stop.is_set():
        msg = m.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"], blocking=True, timeout=1)
        if msg is None:
            continue
        now = time.time()
        if msg.get_type() == "DISTANCE_SENSOR":
            out.put(encode_range(now, msg.current_distance / 100.0, msg.orientation))
            stats["rng"] += 1
            stats["rng_last"] = msg.current_distance / 100.0
        elif msg.get_type() == "LOCAL_POSITION_NED":
            pos, pos_t = (msg.x, msg.y, msg.z), now
        else:
            p = pos if now - pos_t < 0.5 else (math.nan,) * 3
            out.put(encode_pose(now, msg.roll, msg.pitch, msg.yaw, *p))
            stats["att"] += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar", default="/dev/ttyUSB0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="Port, auf dem der Pi auf den Laptop wartet")
    ap.add_argument("--dest", help="festes Ziel IP:PORT statt auf den Laptop zu warten")
    ap.add_argument("--log-dir", default="logs")
    ap.add_argument("--no-log", action="store_true", help="nicht lokal aufzeichnen")
    ap.add_argument("--mav", help="Flugcontroller, z. B. /dev/serial0 (leer = ohne)")
    ap.add_argument("--mav-baud", type=int, default=460800, help="muss zu SERIALx_BAUD passen (bei euch SERIAL6 = 460800)")
    ap.add_argument("--mav-rate", type=float, default=20)
    ap.add_argument("--no-obstacle", action="store_true", help="keine Hindernisdaten an den FC senden")
    ap.add_argument("--lidar-yaw-offset", type=float, default=0.0,
                    help="Grad, um die die 0°-Markierung des LiDARs gegen die Drohnenvorderseite verdreht ist (im Uhrzeigersinn)")
    ap.add_argument("--obstacle-min", type=float, default=0.25, help="m, kürzere Messungen ignorieren (eigene Drohne)")
    ap.add_argument("--obstacle-max", type=float, default=12.0, help="m, maximale Reichweite")
    args = ap.parse_args()

    log_path = None
    if not args.no_log:
        os.makedirs(args.log_dir, exist_ok=True)
        log_path = os.path.join(args.log_dir, time.strftime("scan_%Y%m%d_%H%M%S.bin"))
    out = Output(args.dest, args.port, log_path)
    print(f"Aufzeichnung: {log_path or 'aus'}   "
          + (f"UDP fest an {args.dest}" if args.dest else f"warte auf Laptop (Port {args.port})"))

    stats = {"scans": 0, "points": 0, "att": 0, "rng": 0, "rng_last": None, "cs_err": 0, "format": None,
             "obst": 0, "obst_min": None, "obst_err": None}
    fc = FcLink()
    stop = threading.Event()
    threads = [threading.Thread(target=lidar_loop, args=(args, out, stats, stop, fc), daemon=True)]
    if args.mav:
        threads.append(threading.Thread(target=mav_loop, args=(args, out, stats, stop, fc), daemon=True))
        if not args.no_obstacle:
            threads.append(threading.Thread(target=obstacle_loop, args=(args, stats, stop, fc), daemon=True))
    for th in threads:
        th.start()

    try:
        last = dict(stats)
        while True:
            time.sleep(5)
            out.flush()
            ds = stats["scans"] - last["scans"]
            pts = (stats["points"] - last["points"]) / ds if ds else 0
            line = (f"Scans {ds / 5:.1f}/s ({pts:.0f} Pkt) | Format {stats['format']} Byte"
                    f" | Laptop {'verbunden' if out.connected else 'nicht verbunden'}"
                    f" | Prüfsummenfehler {stats['cs_err']} | UDP-Fehler {out.send_errors}")
            if args.mav:
                line += f" | ATT {(stats['att'] - last['att']) / 5:.0f}/s"
                line += f" | Höhenmesser {(stats['rng'] - last['rng']) / 5:.0f}/s"
                if stats["rng_last"] is not None:
                    line += f" ({stats['rng_last']:.2f} m)"
                if not args.no_obstacle:
                    line += f" | Hindernis->FC {(stats['obst'] - last['obst']) / 5:.0f}/s"
                    if stats["obst_min"] is not None:
                        line += f" (nächstes {stats['obst_min']:.2f} m)"
                    if stats["obst_err"]:
                        line += f" FEHLER: {stats['obst_err']}"
            print(line)
            last = dict(stats)
            if not threads[0].is_alive():
                print("LiDAR-Thread beendet - Abbruch")
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        time.sleep(0.3)
        out.close()
        if log_path:
            print(f"Gespeichert: {log_path}")


if __name__ == "__main__":
    main()
