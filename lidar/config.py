"""Alle Einstellungen an einer Stelle.

Werte hier ändern - die anderen Module lesen sie beim Start.
"""
from pathlib import Path

KARTEN_DIR = Path(__file__).resolve().parent / "karten"   # Ausgaben: karten/<datum_zeit>/

# ======================================================================
# EINSTELLUNGEN
# ======================================================================
MAP_SIZE_M = 60.0        # Kartengröße (quadratisch), Start in der Mitte
RESOLUTION = 0.05        # m pro Zelle
MIN_RANGE = 0.12         # m, kürzere Messungen ignorieren (z. B. eigene Hand)
MIN_SCAN_POINTS = 200    # unvollständige Scans (z. B. der allererste) nicht in die Startkarte
MAX_RANGE = 10.0         # m, T-mini Plus schafft ~12 m
FLIP = False             # True, falls die Karte spiegelverkehrt ist (LiDAR kopfüber)
DESKEW = True            # Bewegungsverzerrung innerhalb eines Scans korrigieren

# Lage vom Flugcontroller (nur wenn der Pi mit --mav läuft)
TILT_COMP = True          # geneigte Scans in die Waagerechte umrechnen
MAX_TILT_DEG = 15.0      # stärker geneigte Scans verwerfen (Ausgleich rechnet bis dahin sauber)
LIDAR_YAW_OFFSET_DEG = 0.0   # Winkel zwischen 0°-Markierung des LiDARs und Drohnen-Vorderseite
LIDAR_ABOVE_RANGEFINDER = 0.0  # m, wie viel höher der LiDAR als der Höhenmesser sitzt
FLOOR_CLEARANCE = 0.10   # m, geneigter Strahl trifft den Boden -> Punkte darunter verwerfen

# Korrelationssuche (normal / nach Verlust)
SEARCH_YAW_DEG = (20.0, 60.0)
SEARCH_XY_M = (0.35, 1.0)
SEARCH_YAW_STEP_DEG = 1.0
FIELD_SIGMA = 0.10       # m, Unschärfe des Wahrscheinlichkeitsfelds

# ICP-Feinabgleich
ICP_MAX_DIST = (0.30, 0.15, 0.08)
ICP_ITER = 25
HUBER = 0.03             # m
MIN_INLIER_RATIO = 0.60  # Anteil Punkte < 10 cm an einer Wand, sonst Scan verwerfen.
                         # Gezählt werden nur Punkte in schon bekannten Bereichen, damit neue
                         # Räume / Fenster / andere Höhen den Scan nicht unnötig verwerfen.
MIN_KNOWN_POINTS = 80    # so viele Punkte müssen in bekannten Bereichen liegen
RETRY_BELOW = 0.80       # Match darunter -> zusätzlich weiträumig suchen, das Bessere nehmen
MIN_MAP_POINTS = 150     # erst ab so vielen Kartenpunkten wird gematcht
START_THRESH = 0.5       # in der Startphase zählt schon eine einmal gesehene Wandzelle
RELOC_AFTER = 12         # Scans (~2 s) ohne Treffer -> Anforderung an den Treffer senken ...
RELAXED_RATIO = 0.40     # ... auf diesen Anteil, damit es nie dauerhaft hängen bleibt

# Occupancy Grid (Log-Odds)
L_OCC, L_FREE = 0.85, -0.4
L_MIN, L_MAX = -4.0, 4.0
L_OCC_THRESH = 1.0       # ab hier gilt eine Zelle als Wand (für das Matching)

# Spiegelfilter: Punkte HINTER einer sicher bekannten Wand, die - an dieser Wand
# zurückgespiegelt - auf einer bekannten Wand landen, sind Spiegelbilder.
MIRROR_FILTER = False    # experimentell, siehe Analyse - standardmäßig aus
L_WALL = 3.5             # so sicher muss die "Wand" sein (L_MAX = 4.0)
MIRROR_BEHIND_MIN = 0.25 # m, Punkt muss mindestens so weit hinter der Wand liegen
MIRROR_CONFIRM = 0.12    # m, zurückgespiegelter Punkt muss so nah an einer Wand landen
MIRROR_MIN_INC = 15.0    # Grad, streifende Strahlen (fast parallel zur Wand) ignorieren
MIRROR_WALL_RADIUS = 0.30   # m, Umgebung zur Prüfung "gerade Fläche"
MIRROR_WALL_MIN_LEN = 0.40  # m, Mindestlänge einer Spiegel-/Wandfläche
MIRROR_CHECK_REFLECTION = False  # Gegenprobe (funktioniert bei geneigtem Spiegel nicht)
MIRROR_CACHE_EVERY = 5      # Wandanalyse alle N Scans aktualisieren
MIRROR_MIN_RUN = 6       # so viele benachbarte Strahlen muss ein Spiegelbild umfassen
WINDOW_PX = 800
DRAW_HZ = 5.0

# ======================================================================
# 3D-Schichten (--3d)
# ======================================================================
LAYER_H = 0.10           # m, Dicke einer Schicht
HEIGHT_STEP = 0.10       # m pro Tastendruck
START_HEIGHT = 0.0       # m, Höhe beim Start
MIN_HITS_LOGODDS = 1.5   # ab hier gilt eine Zelle einer Schicht als belegt (3D-Export)
# ======================================================================

# ======================================================================
# Flugcontroller-Gierwinkel als Vorhersage (abschaltbar mit --no-fc)
# ======================================================================
USE_FC = True            # FC-Gierwinkel nutzen, wenn vorhanden
FC_MAX_DISAGREE_DEG = 25.0   # weicht der FC stärker vom Scan-Matching ab, gilt er als unzuverlässig
