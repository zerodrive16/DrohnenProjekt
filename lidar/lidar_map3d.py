"""3D-Karte aus 2D-LiDAR-Schichten (2,5D) - Test am Laptop.

Idee: Der LiDAR scannt waagerecht auf verschiedenen Höhen. Die Position kommt
aus dem 2D-Scan-Matching von lidar_map.py, die Höhe gibst du per Taste vor
(später: Höhenmesser der Drohne, MTF-01P). Jede Höhe bekommt eine eigene
Belegungskarte (Schicht); zusammen ergibt das eine 3D-Punktwolke.

Benötigt lidar_map.py im selben Ordner.
    pip install pyserial numpy scipy opencv-python matplotlib
    (optional, schönere 3D-Ansicht: pip install open3d)

Start:
    python lidar_map3d.py                       # LiDAR automatisch suchen
    python lidar_map3d.py --port COM5
    python lidar_map3d.py --record raum3d.pkl   # Rohdaten + Höhen aufzeichnen
    python lidar_map3d.py --replay raum3d.pkl --speed 0
    python lidar_map3d.py --pi 100.83.11.79      # LiDAR am Pi (pi_streamer.py läuft dort)
    python lidar_map3d.py --replay scan_XXXX.bin # Pi-Aufzeichnung abspielen
    python lidar_map3d.py --auto 20              # alle 20 s automatisch eine Ebene (+10 cm) höher
    python lidar_map3d.py --auto 15 --step 0.2   # alle 15 s um 20 cm höher
    python lidar_map3d.py --auto 20 --times 5    # bei der 5. Erhöhung Ende (5 Ebenen, 100 s)

Höhe: per Taste (+/-). Läuft der Pi mit Flugcontroller (pi_streamer.py --mav ...),
kommt die Höhe automatisch vom Höhenmesser (MTF-01P), und die Neigung wird
ausgeglichen: jeder Punkt bekommt seine echte Höhe. Ein geneigter Scan trifft
so gleich mehrere Schichten.

So scannst du:
    1. Auf der Starthöhe (z. B. Tischhöhe) einmal langsam durch den Raum.
    2. Höhe ändern, mit + / - die neue Höhe einstellen, wieder durch den Raum.
    3. Wiederholen für weitere Höhen. Beim Höhenwechsel den LiDAR möglichst
       an derselben Stelle halten, damit die Position nicht verloren geht.

Tasten:  + / -  Höhe ±10 cm     p  Höhe = 0 setzen
         v  3D-Ansicht öffnen    s  speichern    q/ESC  Ende (speichert auch)
"""
import argparse
import math
import pickle
import queue
import threading
import time

import cv2
import numpy as np

import lidar_map as lm

# ======================================================================
LAYER_H = 0.10           # m, Dicke einer Schicht
HEIGHT_STEP = 0.10       # m pro Tastendruck
START_HEIGHT = 0.0       # m, Höhe beim Start
MIN_HITS_LOGODDS = 1.5   # ab hier gilt eine Zelle einer Schicht als belegt (3D-Export)
# ======================================================================


class LayerMap:
    """Eine Belegungskarte pro Höhenschicht (gleiche Koordinaten wie die 2D-Karte)."""

    def __init__(self):
        self.layers = {}                         # schicht_index -> GridMap

    def update(self, z, pose, pts_world, zrel=None):
        """z = Höhe des LiDARs, zrel = Höhe jedes Punkts relativ zum LiDAR (bei Neigung)."""
        zp = np.full(len(pts_world), z) if zrel is None else z + zrel
        ks = np.floor(zp / LAYER_H + 1e-9).astype(int)
        for k in np.unique(ks):
            m = ks == k
            if m.sum() < 5:
                continue
            if k not in self.layers:
                self.layers[k] = lm.GridMap(lm.MAP_SIZE_M, lm.RESOLUTION)
                print(f"Neue Schicht: {k * LAYER_H:.2f} - {(k + 1) * LAYER_H:.2f} m")
            self.layers[k].update(pose, pts_world[m])

    def points(self):
        """-> (N,3) Punkte aller belegten Zellen, z = Schichtmitte."""
        out = []
        for k, g in sorted(self.layers.items()):
            ys, xs = np.nonzero(g.L > MIN_HITS_LOGODDS)
            if len(xs) == 0:
                continue
            x = (xs + 0.5) * g.res + g.origin
            y = (ys + 0.5) * g.res + g.origin
            out.append(np.column_stack((x, y, np.full(len(xs), (k + 0.5) * LAYER_H))))
        return np.vstack(out) if out else np.zeros((0, 3))

    def save(self, prefix):
        pts = self.points()
        if len(pts) == 0:
            print("Keine 3D-Punkte zum Speichern.")
            return
        z = pts[:, 2]
        span = max(z.max() - z.min(), 1e-6)
        rgb = cv2.applyColorMap(((z - z.min()) / span * 255).astype(np.uint8)[:, None],
                                cv2.COLORMAP_TURBO)[:, 0, ::-1]
        with open(prefix + ".ply", "w") as f:
            f.write("ply\nformat ascii 1.0\n"
                    f"element vertex {len(pts)}\n"
                    "property float x\nproperty float y\nproperty float z\n"
                    "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                    "end_header\n")
            for (x, y, zz), (r, g, b) in zip(pts, rgb):
                f.write(f"{x:.3f} {y:.3f} {zz:.3f} {r} {g} {b}\n")
        for k, g in self.layers.items():         # jede Schicht auch als Bild
            h = k * LAYER_H
            g.save(f"{prefix}_schicht_{'m' if h < 0 else ''}{abs(h):.2f}m", [])
        print(f"3D-Punktwolke: {prefix}.ply ({len(pts)} Punkte, {len(self.layers)} Schichten)")


def show_3d(points, traj_xy=None):
    """3D-Ansicht: open3d, falls installiert, sonst matplotlib."""
    if len(points) == 0:
        print("Noch keine 3D-Punkte.")
        return
    try:
        import open3d as o3d
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        z = points[:, 2]
        c = cv2.applyColorMap(((z - z.min()) / max(np.ptp(z), 1e-6) * 255).astype(np.uint8)[:, None],
                              cv2.COLORMAP_TURBO)[:, 0, ::-1] / 255.0
        pc.colors = o3d.utility.Vector3dVector(c)
        o3d.visualization.draw_geometries([pc], window_name="3D-Karte")
        return
    except ImportError:
        pass
    import matplotlib.pyplot as plt
    p = points if len(points) < 60000 else points[np.random.default_rng(0).choice(len(points), 60000, False)]
    fig = plt.figure("3D-Karte", figsize=(9, 7))
    ax = fig.add_subplot(projection="3d")
    ax.scatter(p[:, 0], p[:, 1], p[:, 2], c=p[:, 2], cmap="turbo", s=1)
    if traj_xy is not None and len(traj_xy):
        ax.plot(traj_xy[:, 0], traj_xy[:, 1], np.zeros(len(traj_xy)), "k-", lw=0.8)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    # gleiche Achsenskalierung
    mid = (p.max(0) + p.min(0)) / 2
    r = (p.max(0) - p.min(0)).max() / 2
    ax.set_xlim(mid[0] - r, mid[0] + r); ax.set_ylim(mid[1] - r, mid[1] + r)
    ax.set_zlim(max(0, mid[2] - r), mid[2] + r)
    plt.show()


def lidar_thread(port, q, stop, record, height):
    lidar = lm.TMiniPlus(port)
    lidar.start()
    print(f"[lidar] läuft auf {port}")
    try:
        for ang, dist, inten in lidar.scans():
            if stop.is_set():
                break
            z = height[0]
            if record is not None:
                record.append((time.time(), ang, dist, inten, z))
            q.put((ang, dist, z, None))
    finally:
        lidar.stop()


def udp_thread(port, q, stop, record, height, stats, pi=None):
    for t, ang, dist, inten, sens in lm.udp_source(port, stop, stats, pi):
        hz = lm.sensor_height(sens)
        z = hz if hz is not None else height[0]
        if record is not None:
            record.append((t, ang, dist, inten, z))
        q.put((ang, dist, z, sens))


def replay_thread(path, q, stop, speed, height):
    print(f"[replay] {path}")
    t_prev = None
    for t, ang, dist, inten, extra in lm.replay_source(path):
        if stop.is_set():
            return
        if isinstance(extra, dict):                   # Pi-Aufzeichnung: Höhe/Lage vom Flugcontroller
            hz = lm.sensor_height(extra)
            z, sens = (hz if hz is not None else height[0]), extra
        else:                                         # Laptop-Aufzeichnung: gespeicherte Höhe
            z, sens = (extra if extra is not None else START_HEIGHT), None
        if t_prev is not None and speed > 0:
            time.sleep(max(0.0, (t - t_prev) / speed))
        t_prev = t
        q.put((ang, dist, z, sens))
    q.put(None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--record", help="Rohdaten + Höhen aufzeichnen (.pkl)")
    ap.add_argument("--replay", help="Aufzeichnung abspielen (.pkl oder .bin vom Pi)")
    ap.add_argument("--pi", metavar="IP", help="Scans vom Pi holen (Tailscale-IP des Pi)")
    ap.add_argument("--udp", type=int, metavar="PORT", help="Port (Standard 5005)")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--save", help="Dateiname ohne Endung (Standard: karten/<datum_zeit>/karte3d)")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--auto", type=float, metavar="SEK",
                    help="alle SEK Sekunden automatisch eine Ebene höher (statt Taste +)")
    ap.add_argument("--times", type=int, metavar="N",
                    help="mit --auto: bei der N-ten Erhöhung endet der Scan und wird gespeichert "
                         "(= N Ebenen; Standard: unbegrenzt)")
    ap.add_argument("--step", type=float, default=HEIGHT_STEP, metavar="M",
                    help=f"Höhenschritt pro Ebene in m (Standard {HEIGHT_STEP})")
    args = ap.parse_args()
    args.save, args.record = lm.output_paths(args.save, args.record, "karte3d")
    if args.auto and args.replay:
        print("--auto gilt nur live, beim Abspielen kommt die Höhe aus der Aufzeichnung - ignoriert")
        args.auto = None

    height = [START_HEIGHT]                      # Liste, damit der Thread die Änderung sieht
    q = queue.Queue(maxsize=50 if args.replay else 0)
    stop = threading.Event()
    record = [] if args.record else None
    stats = {"lost": 0, "pose": None}
    if args.replay:
        th = threading.Thread(target=replay_thread, args=(args.replay, q, stop, args.speed, height), daemon=True)
    elif args.pi or args.udp:
        args.udp = args.udp or lm.protocol.DEFAULT_PORT
        th = threading.Thread(target=udp_thread, args=(args.udp, q, stop, record, height, stats, args.pi), daemon=True)
    else:
        port = args.port or lm.find_port()
        th = threading.Thread(target=lidar_thread, args=(port, q, stop, record, height), daemon=True)
    th.start()

    mp = lm.Mapper()
    layers = LayerMap()
    t_draw = 0.0
    z = height[0]
    print("Läuft. + / - = Höhe, v = 3D-Ansicht, s = speichern, q = Ende")
    t_level = None                               # Start der aktuellen Ebene (für --auto), ab erstem Scan
    n_auto = 0                                   # so oft wurde automatisch erhöht
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
                    break
                print("Warte auf Scans ...")
                continue
            if item is None:
                break
            ang, dist, z, sens = item
            n_before = len(mp.traj)
            world = mp.process(ang, dist, sens)
            if len(mp.traj) > n_before:          # Scan wurde akzeptiert
                layers.update(z, mp.pose, world, mp.last_zrel)

            now = time.time()
            if t_level is None:
                t_level = now
            last_step = args.times is not None and n_auto + 1 >= args.times
            if args.auto and now - t_level >= args.auto:
                if last_step:                    # letzte Erhöhung fällig -> Scan beenden
                    print(f"\a>>> {args.times}. Erhöhung erreicht - Scan beendet, speichere ...")
                    break
                height[0] = round(height[0] + args.step, 3)
                t_level = now
                n_auto += 1
                print(f"\a>>> Neue Ebene: {height[0]:.2f} m - LiDAR jetzt anheben"
                      + (f" ({n_auto}/{args.times})" if args.times else ""))
                last_step = args.times is not None and n_auto + 1 >= args.times
            if args.no_gui or now - t_draw < 1.0 / lm.DRAW_HZ:
                continue
            t_draw = now
            info = (f"Hoehe {z:.2f} m   Schichten {len(layers.layers)}"
                    f"   match={mp.last_ratio:.0%}   verworfen={mp.n_rejected}")
            if args.auto:
                rest = max(0.0, args.auto - (now - t_level))
                info += f"   {'Ende' if last_step else 'naechste Ebene'} in {rest:.0f} s"
            cv2.imshow("LiDAR-Karte 3D (Draufsicht)", lm.draw(mp.grid, mp.pose, mp.traj, world, info))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("+"), ord("=")):
                height[0] = round(height[0] + args.step, 3)
                t_level = now                    # Auto-Zähler neu starten
                print(f"Höhe: {height[0]:.2f} m")
            if key == ord("-"):
                height[0] = round(height[0] - args.step, 3)
                t_level = now
                print(f"Höhe: {height[0]:.2f} m")
            if key == ord("p"):
                height[0] = 0.0
                t_level = now
                print("Höhe: 0.00 m")
            if key == ord("s"):
                mp.grid.save(args.save + "_2d", mp.traj)
                layers.save(args.save)
            if key == ord("v"):
                show_3d(layers.points(), np.array(mp.traj)[:, :2] if mp.traj else None)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print(f"{mp.n_scans} Scans, {mp.n_rejected} verworfen")
        mp.grid.save(args.save + "_2d", mp.traj)
        layers.save(args.save)
        if record:
            with open(args.record, "wb") as f:
                pickle.dump(record, f)
            print(f"Rohdaten: {args.record} ({len(record)} Scans)")
        cv2.destroyAllWindows()
        if not args.no_gui and layers.layers:
            show_3d(layers.points(), np.array(mp.traj)[:, :2] if mp.traj else None)


if __name__ == "__main__":
    main()
