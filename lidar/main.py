"""LiDAR-Kartierung (2D und 3D) - einziger Einstiegspunkt.

Installation:
    pip install pyserial numpy scipy opencv-python matplotlib
    (optional, schönere 3D-Ansicht: pip install open3d)

Datenquelle (eine davon):
    python main.py                            LiDAR per USB am Laptop (Port wird gesucht)
    python main.py --port COM5                LiDAR per USB, Port selbst angeben
    python main.py --pi 10.42.0.1             LiDAR am Pi (dort läuft pi_streamer.py)
    python main.py --replay scan_XXXX.bin     Aufzeichnung abspielen (.bin vom Pi, .pkl vom Laptop)

Optionen:
    --3d                  3D-Karte aus Höhenschichten (sonst 2D)
    --auto SEK            (3D) alle SEK Sekunden eine Ebene höher, ohne Höhenmesser
    --step M              (3D) Höhenschritt pro Ebene, Standard 0.10 m
    --times N             (3D, mit --auto) bei der N-ten Erhöhung endet der Scan und wird gespeichert
    --no-fc               Gierwinkel des Flugcontrollers NICHT nutzen (Standard: nutzen, wenn vorhanden)
    --record DATEI.pkl    Rohdaten aufzeichnen (bei LiDAR am Pi zeichnet der Pi ohnehin auf)
    --save NAME           Dateiname; Standard: karten/<datum_zeit>/karte bzw. karte3d
    --speed X             Abspielgeschwindigkeit (0 = so schnell wie möglich)
    --no-gui              ohne Fenster (z. B. zum Auswerten von Aufnahmen)

Tasten im Fenster:
    s = speichern    r = Karte zurücksetzen    q/ESC = Ende (speichert auch)
    3D zusätzlich:   + / - = Höhe ±Schritt    p = Höhe 0    v = 3D-Ansicht
"""
import argparse
import pickle
import queue
import threading
import time

import cv2
import numpy as np

import protocol
from config import DRAW_HZ, HEIGHT_STEP, START_HEIGHT, USE_FC
from maps import LayerMap, output_paths
from slam import Mapper, sensor_height
from sources import find_port, replay_source, udp_source, usb_source
from viz import draw, show_3d


def source_thread(gen, q, stop, speed=None):
    """Scans aus einem Generator in die Warteschlange legen (beim Abspielen im Originaltakt)."""
    t_prev = None
    n = 0
    for t, ang, dist, inten, extra in gen:
        if stop.is_set():
            return
        if speed is not None:
            if t_prev is not None and speed > 0:
                time.sleep(max(0.0, (t - t_prev) / speed))
            t_prev = t
        q.put((t, ang, dist, inten, extra))
        n += 1
    if speed is not None:
        print(f"[replay] {n} Scans abgespielt")
    q.put(None)


def parse_args():
    ap = argparse.ArgumentParser(description="LiDAR-Kartierung (2D/3D)")
    ap.add_argument("--port", help="LiDAR per USB, z. B. COM5 oder /dev/ttyUSB0")
    ap.add_argument("--pi", metavar="IP", help="Scans vom Pi holen (dort läuft pi_streamer.py)")
    ap.add_argument("--udp", type=int, metavar="PORT", help="UDP-Port (Standard 5005); ohne --pi: warten, bis der Pi sendet")
    ap.add_argument("--replay", help="Aufzeichnung abspielen (.bin vom Pi oder .pkl vom Laptop)")
    ap.add_argument("--speed", type=float, default=1.0, help="Abspielgeschwindigkeit (0 = so schnell wie möglich)")
    ap.add_argument("--record", help="Rohdaten in diese Datei aufzeichnen (.pkl)")
    ap.add_argument("--save", help="Dateiname ohne Endung (Standard: karten/<datum_zeit>/karte bzw. karte3d)")
    ap.add_argument("--no-gui", action="store_true", help="ohne Fenster")
    ap.add_argument("--3d", dest="three_d", action="store_true", help="3D-Karte aus Höhenschichten")
    ap.add_argument("--auto", type=float, metavar="SEK", help="(3D) alle SEK Sekunden eine Ebene höher")
    ap.add_argument("--step", type=float, default=HEIGHT_STEP, metavar="M",
                    help=f"(3D) Höhenschritt pro Ebene in m (Standard {HEIGHT_STEP})")
    ap.add_argument("--times", type=int, metavar="N",
                    help="(3D, mit --auto) bei der N-ten Erhöhung endet der Scan und wird gespeichert "
                         "(= N Ebenen; Standard: unbegrenzt)")
    ap.add_argument("--no-fc", action="store_true", help="Gierwinkel des Flugcontrollers nicht nutzen")
    args = ap.parse_args()
    if args.auto and not args.three_d:
        print("--auto gehört zur 3D-Karte - schalte --3d ein")
        args.three_d = True
    if args.auto and args.replay:
        print("--auto gilt nur live, beim Abspielen kommt die Höhe aus der Aufzeichnung - ignoriert")
        args.auto = None
    return args


def main():
    args = parse_args()
    args.save, args.record = output_paths(args.save, args.record, "karte3d" if args.three_d else "karte")

    q = queue.Queue(maxsize=50 if args.replay else 0)
    stop = threading.Event()
    stats = {"lost": 0, "pose": None}
    if args.replay:
        gen, speed = replay_source(args.replay), args.speed
    elif args.pi or args.udp:
        args.udp = args.udp or protocol.DEFAULT_PORT
        gen, speed = udp_source(args.udp, stop, stats, args.pi), None
    else:
        gen, speed = usb_source(args.port or find_port(), stop), None
    th = threading.Thread(target=source_thread, args=(gen, q, stop, speed), daemon=True)
    th.start()

    use_fc = USE_FC and not args.no_fc
    mp = Mapper(use_fc=use_fc)
    layers = LayerMap() if args.three_d else None
    record = [] if args.record else None
    height = START_HEIGHT                        # Höhe per Taste/--auto (3D, ohne Höhenmesser)
    t_draw, t_last, hz = 0.0, time.time(), 0.0
    t_level = None                               # Start der aktuellen Ebene (für --auto)
    n_auto = 0                                   # so oft wurde automatisch erhöht
    last_step = False
    z = height
    print("Läuft. s = speichern, r = zurücksetzen, q = Ende"
          + (", + / - = Höhe, v = 3D-Ansicht" if args.three_d else ""))
    if args.auto:
        print(f"Auto-Ebenen: alle {args.auto:g} s um {args.step:.2f} m höher"
              + (f", {args.times}-mal" if args.times else "")
              + " (gilt nur, solange kein Höhenmesser die Höhe liefert)")

    try:
        while True:
            try:
                item = q.get(timeout=3)
            except queue.Empty:
                if not th.is_alive():
                    print("Datenquelle beendet.")
                    break
                print("Warte auf Scans ... " + ("(läuft pi_streamer.py auf dem Pi? richtige IP?)" if args.udp
                                                else "(LiDAR angeschlossen und dreht sich?)"))
                continue
            if item is None:
                break
            t, ang, dist, inten, extra = item
            sens = extra if isinstance(extra, dict) else None

            # Höhe dieses Scans (nur 3D): Höhenmesser > gespeicherte Höhe (.pkl) > Taste/--auto
            if args.three_d:
                hz_sens = sensor_height(sens)
                if hz_sens is not None:
                    z = hz_sens
                elif args.replay and sens is None:
                    z = extra if extra is not None else START_HEIGHT
                else:
                    z = height
            if record is not None:
                record.append((t, ang, dist, inten, z if args.three_d else None))

            n_before = len(mp.traj)
            world = mp.process(ang, dist, sens)
            if layers is not None and len(mp.traj) > n_before:   # angenommener Scan -> 3D-Schichten
                layers.update(z, mp.pose, world, mp.last_zrel)

            now = time.time()
            hz = 0.9 * hz + 0.1 / max(now - t_last, 1e-3)
            t_last = now
            if t_level is None:
                t_level = now
            last_step = args.times is not None and n_auto + 1 >= args.times
            if args.auto and now - t_level >= args.auto:
                if last_step:                    # letzte Erhöhung fällig -> Scan beenden
                    print(f"\a>>> {args.times}. Erhöhung erreicht - Scan beendet, speichere ...")
                    break
                height = round(height + args.step, 3)
                t_level = now
                n_auto += 1
                print(f"\a>>> Neue Ebene: {height:.2f} m - LiDAR jetzt anheben"
                      + (f" ({n_auto}/{args.times})" if args.times else ""))
                last_step = args.times is not None and n_auto + 1 >= args.times
            if args.no_gui or now - t_draw < 1.0 / DRAW_HZ:
                continue
            t_draw = now

            backlog = q.qsize() if not args.replay else 0
            state = "VERLOREN - langsam zurück zu bekanntem Bereich" if mp.lost > 3 else ""
            tilt = f"  Neigung={mp.last_tilt:.0f}deg (verw. {mp.n_tilt})" if mp.last_tilt is not None else ""
            fc = "  FC-Gier" if (use_fc and mp.fc_yaw_ref is not None) else ""
            info = (f"match={mp.last_ratio:.0%}  {hz:.1f} Hz  verworfen={mp.n_rejected}{tilt}{fc}"
                    + (f"  Rueckstand={backlog}" if backlog > 2 else "")
                    + (f"  UDP-Verlust={stats['lost']}" if args.udp else ""))
            if args.three_d:
                info = f"Hoehe {z:.2f} m  Schichten {len(layers.layers)}  " + info
                if args.auto:
                    rest = max(0.0, args.auto - (now - t_level))
                    info += f"  {'Ende' if last_step else 'naechste Ebene'} in {rest:.0f} s"
            info += f"  {state}"
            title = "LiDAR-Karte 3D (Draufsicht)" if args.three_d else "LiDAR-Karte"
            cv2.imshow(title, draw(mp.grid, mp.pose, mp.traj, world, info))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                save(args, mp, layers)
            if key == ord("r"):
                mp = Mapper(use_fc=use_fc)
                layers = LayerMap() if args.three_d else None
                print("Karte zurückgesetzt")
            if args.three_d:
                if key in (ord("+"), ord("=")):
                    height = round(height + args.step, 3)
                    t_level = now
                    print(f"Höhe: {height:.2f} m")
                if key == ord("-"):
                    height = round(height - args.step, 3)
                    t_level = now
                    print(f"Höhe: {height:.2f} m")
                if key == ord("p"):
                    height = 0.0
                    t_level = now
                    print("Höhe: 0.00 m")
                if key == ord("v"):
                    show_3d(layers.points(), np.array(mp.traj)[:, :2] if mp.traj else None)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print(f"{mp.n_scans} Scans verarbeitet, {mp.n_rejected} verworfen, {mp.n_tilt} wegen Neigung verworfen"
              + (f", FC-Gierwinkel bei {mp.n_fc_used} Scans genutzt" if mp.n_fc_used else ""))
        save(args, mp, layers)
        if record:
            with open(args.record, "wb") as f:
                pickle.dump(record, f)
            print(f"Rohdaten: {args.record} ({len(record)} Scans)")
        cv2.destroyAllWindows()
        if args.three_d and not args.no_gui and layers.layers:
            show_3d(layers.points(), np.array(mp.traj)[:, :2] if mp.traj else None)


def save(args, mp, layers):
    if layers is None:
        mp.grid.save(args.save, mp.traj)
    else:
        mp.grid.save(args.save + "_2d", mp.traj)
        layers.save(args.save)


if __name__ == "__main__":
    main()
