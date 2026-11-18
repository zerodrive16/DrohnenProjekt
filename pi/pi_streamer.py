"""Läuft auf dem Raspberry Pi: LiDAR lesen -> per UDP an den Laptop + lokal aufzeichnen.

Dateien auf dem Pi (alle im selben Ordner):
    pi_streamer.py, tmini_plus.py, protocol.py
Installation auf dem Pi:
    pip3 install pyserial              (mit Flugcontroller zusätzlich: pip3 install pymavlink)

Start (normal): Pi wartet, bis sich der Laptop meldet
    python3 pi_streamer.py
    Laptop:  python lidar_map.py --pi PI_IP          (PI_IP = Tailscale-IP des Pi)

    python3 pi_streamer.py --mav /dev/serial0 --mav-baud 115200
                                                    # + Lage/Höhe vom Flugcontroller
Alternativ (nur wenn der Pi den Laptop selbst erreichen darf):
    python3 pi_streamer.py --dest LAPTOP_IP:5005    Laptop: python lidar_map.py --udp 5005

Aufzeichnungen landen in logs/scan_JJJJMMTT_HHMMSS.bin. Am Laptop abspielen mit:
    python lidar_map.py --replay scan_....bin
"""
import argparse
import math
import os
import socket
import threading
import time

from protocol import DEFAULT_PORT, HELLO, LEN, encode_pose, encode_scan
from tmini_plus import TMiniPlus


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


def lidar_loop(args, out, stats, stop):
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
            scan_id += 1
            stats["scans"] += 1
            stats["points"] += len(angles)
            stats["cs_err"] = lidar.cs_errors
            stats["format"] = lidar.sample_size
    finally:
        lidar.stop()


def mav_loop(args, out, stats, stop):
    from pymavlink import mavutil

    print(f"[mav] verbinde mit {args.mav} ...")
    m = mavutil.mavlink_connection(args.mav, baud=args.mav_baud)
    if m.wait_heartbeat(timeout=15) is None:
        print("[mav] kein Heartbeat vom Flugcontroller - nur LiDAR wird gesendet")
        return
    print(f"[mav] Heartbeat von System {m.target_system}")
    for msg_id in (30, 32):  # ATTITUDE, LOCAL_POSITION_NED
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, int(1e6 / args.mav_rate), 0, 0, 0, 0, 0)
    pos, pos_t = (math.nan,) * 3, 0.0
    while not stop.is_set():
        msg = m.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED"], blocking=True, timeout=1)
        if msg is None:
            continue
        now = time.time()
        if msg.get_type() == "LOCAL_POSITION_NED":
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
    ap.add_argument("--mav-baud", type=int, default=115200, help="muss zu SERIALx_BAUD passen")
    ap.add_argument("--mav-rate", type=float, default=20)
    args = ap.parse_args()

    log_path = None
    if not args.no_log:
        os.makedirs(args.log_dir, exist_ok=True)
        log_path = os.path.join(args.log_dir, time.strftime("scan_%Y%m%d_%H%M%S.bin"))
    out = Output(args.dest, args.port, log_path)
    print(f"Aufzeichnung: {log_path or 'aus'}   "
          + (f"UDP fest an {args.dest}" if args.dest else f"warte auf Laptop (Port {args.port})"))

    stats = {"scans": 0, "points": 0, "att": 0, "cs_err": 0, "format": None}
    stop = threading.Event()
    threads = [threading.Thread(target=lidar_loop, args=(args, out, stats, stop), daemon=True)]
    if args.mav:
        threads.append(threading.Thread(target=mav_loop, args=(args, out, stats, stop), daemon=True))
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
