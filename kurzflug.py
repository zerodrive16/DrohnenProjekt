#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kurzflug.py - kurzer, abgesicherter Testflug fuer ArduCopter
(Origin setzen, EKF-Quelle waehlen, GUIDED, Takeoff, Schweben, Landen)

Laeuft auf dem Raspberry Pi und spricht ueber MAVProxy mit dem Flight Controller.

Vorbereitung:
  1. MAVProxy wie gewohnt starten:
       mavproxy.py --master=/dev/serial0 --baudrate 115200
  2. Im MAVProxy-Prompt einen lokalen Ausgang fuer dieses Skript anlegen:
       output add 127.0.0.1:14560
  3. In einem zweiten Terminal (zweites SSH-Fenster):
       python3 kurzflug.py --dry-run     # Trockenlauf, die Drohne wird NICHT scharf
       python3 kurzflug.py               # echter Testflug

Sicherheit:
  - Sender in der Hand, Modusschalter (CH6) als Rueckfall auf Stabilize.
    Wechselt der Modus von GUIDED weg, beendet das Skript sich sofort
    und sendet NICHTS mehr (der Pilot hat Vorrang).
  - Hoehe ist auf 3 m begrenzt. Bei zu grosser Hoehe, niedrigem Akku oder
    Verbindungsverlust wird LAND gesendet.
  - Strg+C sendet LAND.
  - Tischtest nur ohne Propeller, Flug nur auf freier Flaeche.
"""
import argparse
import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

mavlink = mavutil.mavlink
MAX_ALT_M = 3.0
SET_EKF_SOURCE_SET = getattr(mavlink, "MAV_CMD_SET_EKF_SOURCE_SET", 42007)


class FC:
    """Verbindung zum Flight Controller und zuletzt bekannter Zustand."""

    def __init__(self, connect):
        self.m = mavutil.mavlink_connection(connect)
        self.mode = None
        self.armed = False
        self.alt = None      # Hoehe in m (aus LOCAL_POSITION_NED)
        self.batt = -1       # Akku in Prozent, -1 = unbekannt
        self.last_hb = 0.0
        self.acks = {}

    def handle(self, msg):
        t = msg.get_type()
        if t == "HEARTBEAT":
            if (msg.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                    and msg.type != mavlink.MAV_TYPE_GCS):
                self.mode = mavutil.mode_string_v10(msg)
                self.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.last_hb = time.time()
        elif t == "STATUSTEXT":
            print("  FC: %s" % msg.text)
        elif t == "LOCAL_POSITION_NED":
            self.alt = -msg.z
        elif t == "SYS_STATUS":
            self.batt = msg.battery_remaining
        elif t == "COMMAND_ACK":
            self.acks[msg.command] = msg.result

    def pump(self, seconds):
        """Nachrichten fuer 'seconds' Sekunden lesen und den Zustand aktualisieren."""
        end = time.time() + seconds
        while time.time() < end:
            msg = self.m.recv_match(blocking=True, timeout=0.1)
            if msg is not None:
                self.handle(msg)

    def wait_fc(self, timeout=15):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if (msg is not None
                    and msg.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                    and msg.type != mavlink.MAV_TYPE_GCS):
                self.m.target_system = msg.get_srcSystem()
                self.m.target_component = msg.get_srcComponent()
                self.handle(msg)
                self.m.mav.request_data_stream_send(
                    self.m.target_system, self.m.target_component,
                    mavlink.MAV_DATA_STREAM_ALL, 4, 1)
                return True
        return False

    def command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, wait=3.0):
        """COMMAND_LONG senden. True, wenn der FC ihn akzeptiert hat."""
        self.acks.pop(cmd, None)
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            cmd, 0, p1, p2, p3, p4, p5, p6, p7)
        end = time.time() + wait
        while time.time() < end:
            self.pump(0.1)
            if cmd in self.acks:
                return self.acks[cmd] in (mavlink.MAV_RESULT_ACCEPTED,
                                          mavlink.MAV_RESULT_IN_PROGRESS)
        return False

    def set_mode(self, name, timeout=5.0):
        self.m.set_mode(name)
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.2)
            if self.mode == name:
                return True
        return False

    def set_origin(self, lat, lon, alt_m):
        self.m.mav.set_gps_global_origin_send(
            self.m.target_system, int(lat * 1e7), int(lon * 1e7), int(alt_m * 1000))
        self.pump(1.0)


def check_safe(fc, a):
    """Gibt None zurueck, wenn alles in Ordnung ist, sonst einen Grund."""
    if fc.mode != "GUIDED":
        return "PILOT"
    if time.time() - fc.last_hb > 3.0:
        return "VERBINDUNG weg"
    if fc.alt is not None and fc.alt > a.alt + 1.0:
        return "ZU HOCH (%.1f m)" % fc.alt
    if 0 <= fc.batt < 25:
        return "AKKU niedrig (%d %%)" % fc.batt
    return None


def land(fc, timeout=40.0):
    print("Landen ...")
    fc.set_mode("LAND")
    end = time.time() + timeout
    while fc.armed and time.time() < end:
        fc.pump(0.3)
    if fc.armed:
        print("Noch nicht gelandet - bitte mit dem Sender uebernehmen!")
    else:
        print("Gelandet und disarmed.")


def fly(fc, a):
    """Der eigentliche Testflug. Gibt True zurueck, wenn der Pilot uebernommen hat."""
    print("Arme ...")
    fc.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    end = time.time() + 5.0
    while not fc.armed and time.time() < end:
        fc.pump(0.2)
    if not fc.armed:
        print("Arming fehlgeschlagen - siehe Meldungen des FC oben. Abbruch.")
        return False

    print("Takeoff auf %.1f m ..." % a.alt)
    if not fc.command(mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, a.alt):
        print("Takeoff abgelehnt - disarme.")
        fc.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        return False

    t0 = time.time()
    reached = False
    while time.time() - t0 < a.climb_timeout:
        fc.pump(0.2)
        reason = check_safe(fc, a)
        if reason == "PILOT":
            print("Modus nicht mehr GUIDED - Pilot hat uebernommen. Skript endet.")
            return True
        if reason:
            print("Abbruch: %s" % reason)
            land(fc)
            return False
        if fc.alt is not None and fc.alt >= 0.9 * a.alt:
            reached = True
            break
    if not reached:
        print("Zielhoehe nicht erreicht (Hoehe: %s) -> LAND" % fc.alt)
        land(fc)
        return False

    print("Schwebe %.0f s auf %.1f m ..." % (a.hover, fc.alt))
    t0 = time.time()
    while time.time() - t0 < a.hover:
        fc.pump(0.2)
        reason = check_safe(fc, a)
        if reason == "PILOT":
            print("Modus nicht mehr GUIDED - Pilot hat uebernommen. Skript endet.")
            return True
        if reason:
            print("Abbruch: %s" % reason)
            land(fc)
            return False
    land(fc)
    return False


def parse_args():
    p = argparse.ArgumentParser(description="Kurzer abgesicherter Testflug (ArduCopter, GUIDED)")
    p.add_argument("--connect", default="udpin:127.0.0.1:14560",
                   help="MAVLink-Verbindung (Standard: Ausgang von MAVProxy)")
    p.add_argument("--alt", type=float, default=1.5, help="Zielhoehe in m (0.5 bis 3.0)")
    p.add_argument("--hover", type=float, default=5.0, help="Schwebedauer in s")
    p.add_argument("--climb-timeout", type=float, default=15.0, help="max. Steigzeit in s")
    p.add_argument("--source-set", type=int, default=2, choices=(1, 2, 3),
                   help="EKF-Quellensatz (2 = Fluss+Baro, 3 = Fluss+Lidar)")
    p.add_argument("--lat", type=float, default=49.6, help="EKF-Origin Breitengrad")
    p.add_argument("--lon", type=float, default=11.0, help="EKF-Origin Laengengrad")
    p.add_argument("--origin-alt", type=float, default=300.0, help="EKF-Origin Hoehe in m")
    p.add_argument("--no-origin", action="store_true", help="EKF-Origin nicht setzen")
    p.add_argument("--min-batt", type=int, default=50, help="Mindest-Akkustand in Prozent")
    p.add_argument("--dry-run", action="store_true",
                   help="Trockenlauf: alles vorbereiten, aber NICHT scharf schalten")
    return p.parse_args()


def main():
    a = parse_args()
    if not (0.5 <= a.alt <= MAX_ALT_M):
        sys.exit("Hoehe muss zwischen 0.5 und %.1f m liegen." % MAX_ALT_M)

    fc = FC(a.connect)
    print("Warte auf den Flight Controller ...")
    if not fc.wait_fc():
        sys.exit("Kein Heartbeat. Laeuft MAVProxy, und wurde 'output add 127.0.0.1:14560' eingegeben?")

    fc.pump(2.0)
    print("Modus: %s | scharf: %s | Akku: %s %%" % (
        fc.mode, fc.armed, fc.batt if fc.batt >= 0 else "?"))
    if fc.armed:
        sys.exit("Die Drohne ist schon scharf - bitte erst disarmen.")
    if 0 <= fc.batt < a.min_batt:
        sys.exit("Akku unter %d %% - Abbruch." % a.min_batt)

    if not a.no_origin:
        print("Setze EKF-Origin (%.4f, %.4f, %.0f m) ..." % (a.lat, a.lon, a.origin_alt))
        fc.set_origin(a.lat, a.lon, a.origin_alt)

    print("Waehle EKF-Quellensatz %d ..." % a.source_set)
    if not fc.command(SET_EKF_SOURCE_SET, a.source_set):
        print("  Warnung: Quellenumschaltung nicht bestaetigt.")
    fc.pump(1.0)

    print("Wechsle in GUIDED ...")
    if not fc.set_mode("GUIDED"):
        sys.exit("GUIDED wurde nicht angenommen (Positionsschaetzung? Meldungen des FC oben).")

    if a.dry_run:
        print("Trockenlauf beendet: Verbindung, Origin, Quellensatz und GUIDED sind in Ordnung.")
        return

    print()
    print("ACHTUNG: Gleich werden die Motoren scharf und die Drohne hebt auf %.1f m ab." % a.alt)
    print("  - Propeller nur montiert, wenn eine freie Flaeche vorhanden ist.")
    print("  - Sender in der Hand, Schalter CH6 zum Uebernehmen (Stabilize).")
    if input("Zum Starten FLIEGEN eintippen: ").strip() != "FLIEGEN":
        print("Abgebrochen.")
        return

    handed_over = False
    try:
        handed_over = fly(fc, a)
    except BaseException as exc:  # auch Strg+C
        print("\nUnterbrochen (%s) -> LAND" % type(exc).__name__)
        if fc.armed and fc.mode == "GUIDED":
            land(fc)
        raise
    finally:
        if not handed_over and fc.armed and fc.mode == "GUIDED":
            land(fc)


if __name__ == "__main__":
    main()
