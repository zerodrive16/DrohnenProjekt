"""
Erster Guided-Test für die Indoor-Drohne mit ArduPilot.

Zwei Varianten:
  A) --pilot-takeoff:
     Pilot armt und startet selbst in Loiter, schaltet dann per Schalter auf GUIDED.
     Das Skript fliegt 0,5 m vorwärts, wieder zurück und landet.
  B) ohne --pilot-takeoff:
     Skript setzt GUIDED, armt, startet auf 1 m, fliegt 0,5 m vorwärts und landet.

Zusätzlich:
  --set-origin LAT LON  setzt das EKF-Origin (nötig ohne GPS, siehe Tutorial Schritt 6)
  --dry-run             nur verbinden + Status zeigen, nichts armen, nichts fliegen

SICHERHEIT:
  * Erst mit --dry-run und OHNE Propeller testen.
  * Pilot hält die Fernsteuerung IMMER in der Hand und kann per Schalter
    sofort in Loiter/AltHold wechseln oder den Not-Aus auslösen.
    Ein Moduswechsel weg von GUIDED beendet dieses Skript sofort.
  * Nur in abgesichertem Bereich mit Propellerschutz/Netz fliegen.
  * Ctrl+C im Terminal -> Skript schickt LAND (nur solange GUIDED aktiv ist).

Voraussetzungen:
  pip install pymavlink

Beispiele:
  python guided_test.py --connect udpin:0.0.0.0:14551 --dry-run
  python guided_test.py --connect udpin:0.0.0.0:14551 --set-origin 50.13 8.69 --dry-run
  python guided_test.py --connect udpin:0.0.0.0:14551 --pilot-takeoff
"""

import argparse
import sys
import time

from pymavlink import mavutil

# SET_POSITION_TARGET_LOCAL_NED: nur Position verwenden
# (Geschwindigkeit, Beschleunigung, Yaw und Yaw-Rate ignorieren)
TYPE_MASK_POS_ONLY = 0b0000_1101_1111_1000


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def current_mode(m):
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    return mavutil.mode_string_v10(hb) if hb else None


def set_mode(m, mode):
    m.set_mode(mode)
    t0 = time.time()
    while time.time() - t0 < 5:
        if current_mode(m) == mode:
            log(f"Modus: {mode}")
            return
    raise RuntimeError(f"Moduswechsel auf {mode} fehlgeschlagen "
                       "(EKF-Origin gesetzt? Position vorhanden?)")


def check_still_guided(m):
    """Bricht ab, wenn der Pilot per Fernsteuerung den Modus gewechselt hat."""
    mode = current_mode(m)
    if mode and mode != "GUIDED":
        raise KeyboardInterrupt(f"Pilot hat übernommen (Modus {mode})")


def local_position(m, timeout=2):
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
    return (msg.x, msg.y, msg.z) if msg else None


def request_local_position(m, hz=10):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, int(1e6 / hz), 0, 0, 0, 0, 0)


def set_origin(m, lat, lon, alt_m=0.0):
    m.mav.set_gps_global_origin_send(
        m.target_system, int(lat * 1e7), int(lon * 1e7), int(alt_m * 1000))
    log(f"EKF-Origin gesendet: {lat}, {lon}")
    # Kurz warten und Rückmeldung des FC ausgeben (GPS_GLOBAL_ORIGIN / STATUSTEXT)
    t0 = time.time()
    while time.time() - t0 < 3:
        msg = m.recv_match(type=["GPS_GLOBAL_ORIGIN", "STATUSTEXT"], blocking=True, timeout=1)
        if msg:
            log(f"FC: {msg}")


def wait_altitude(m, target_up_m, tol=0.15, timeout=15):
    t0 = time.time()
    while time.time() - t0 < timeout:
        check_still_guided(m)
        pos = local_position(m)
        if pos:
            up = -pos[2]                       # NED: z zeigt nach unten
            log(f"Höhe {up:.2f} m")
            if abs(up - target_up_m) < tol:
                return
    raise RuntimeError("Zielhöhe nicht erreicht")


def goto_local(m, north, east, up):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, TYPE_MASK_POS_ONLY,
        north, east, -up, 0, 0, 0, 0, 0, 0, 0, 0)


def hold(m, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        check_still_guided(m)
        pos = local_position(m)
        if pos:
            log(f"Position N {pos[0]:.2f}  E {pos[1]:.2f}  oben {-pos[2]:.2f}")


def wait_for_pilot_guided(m, timeout=120):
    log("Warte, bis der Pilot in der Luft auf GUIDED schaltet ...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        if current_mode(m) == "GUIDED":
            log("GUIDED erkannt – Skript übernimmt")
            return
    raise RuntimeError("Pilot hat nicht auf GUIDED geschaltet")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--connect", default="udpin:0.0.0.0:14551")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--alt", type=float, default=1.0, help="Abhebehöhe in m (Variante B)")
    p.add_argument("--forward", type=float, default=0.5, help="Strecke vorwärts in m")
    p.add_argument("--set-origin", nargs=2, type=float, metavar=("LAT", "LON"),
                   help="EKF-Origin setzen (beliebige feste Koordinate, z. B. Gebäude)")
    p.add_argument("--pilot-takeoff", action="store_true",
                   help="Pilot startet selbst und schaltet auf GUIDED (Variante A)")
    p.add_argument("--dry-run", action="store_true",
                   help="nur verbinden und Status zeigen – nichts armen, nichts fliegen")
    a = p.parse_args()

    m = mavutil.mavlink_connection(a.connect, baud=a.baud)
    log("Warte auf Heartbeat ...")
    m.wait_heartbeat(timeout=15)
    log(f"Verbunden mit System {m.target_system}, Komponente {m.target_component}")

    if a.set_origin:
        set_origin(m, *a.set_origin)

    request_local_position(m)
    pos = local_position(m, timeout=3)
    if pos is None:
        log("WARNUNG: keine LOCAL_POSITION_NED – EKF hat keine lokale Position. "
            "EKF-Origin setzen (Tutorial Schritt 6) und EKF-Source-Set 3 prüfen.")
    else:
        log(f"Lokale Position vorhanden: N {pos[0]:.2f} E {pos[1]:.2f} oben {-pos[2]:.2f}")
    log(f"Aktueller Modus: {current_mode(m)}")

    if a.dry_run:
        log("Dry-Run fertig – nichts wurde gearmt oder geflogen.")
        return

    if input("Propeller frei, Pilot bereit? 'ja' eingeben: ").strip().lower() != "ja":
        sys.exit("Abgebrochen.")

    try:
        if a.pilot_takeoff:
            wait_for_pilot_guided(m)
            pos = local_position(m)
            up = -pos[2]
        else:
            set_mode(m, "GUIDED")
            m.arducopter_arm()
            m.motors_armed_wait()
            log("Gearmt")
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, a.alt)
            wait_altitude(m, a.alt)
            up = a.alt

        hold(m, 3)
        start = local_position(m)
        log(f"Fliege {a.forward} m nach Norden (lokales Koordinatensystem)")
        goto_local(m, start[0] + a.forward, start[1], up)
        hold(m, 5)
        log("Zurück zum Startpunkt")
        goto_local(m, start[0], start[1], up)
        hold(m, 5)

    except (KeyboardInterrupt, RuntimeError) as e:
        log(f"Abbruch: {e}")
    finally:
        mode = current_mode(m)
        if mode == "GUIDED":
            log("Lande ...")
            m.set_mode("LAND")
        else:
            log(f"Modus ist {mode} – Pilot hat Kontrolle, kein LAND gesendet")


if __name__ == "__main__":
    main()