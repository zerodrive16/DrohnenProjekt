# LiDAR-Kartierung (2D und 2,5D)

Aus dem **YDLIDAR T-mini Plus** entsteht eine Karte. Die Position (x, y, Drehung) wird allein
aus den Scans geschätzt (Scan-Matching / SLAM), ohne GPS. Läuft der Pi mit Flugcontroller,
kommen **Höhe** (Höhenmesser MTF-01P) und **Neigung** (Roll/Pitch) vom FC dazu.

| Datei | Läuft auf | Aufgabe |
|---|---|---|
| `lidar_map.py` | Laptop | 2D-Karte: LiDAR-Treiber, Datenquellen, Neigungsausgleich, Scan-Matching, Occupancy Grid |
| `lidar_map3d.py` | Laptop | 2,5D-Karte: eine 2D-Karte pro Höhenschicht → 3D-Punktwolke (`.ply`) |
| `protocol.py` | beide | Nachrichtenformat Pi ↔ Laptop, Log-Format `.bin`, Zuordnung der FC-Werte zu den Scans |
| `pi/pi_streamer.py` | Pi | LiDAR + Flugcontroller lesen, per UDP senden, als `.bin` aufzeichnen |
| `pi/tmini_plus.py` | Pi | LiDAR-Treiber ohne numpy (für den Pi) |

```
 Pi ─────────────────────────────────────────────┐        Laptop ──────────────────────────────────────────
 tmini_plus  ── Scans ──┐                         │
                        ├─► pi_streamer ── UDP ───┼──► udp_source ─┐
 Flugcontroller (MAVLink)                         │                │   SensorState.at(t)
   ATTITUDE          → T_POSE ┘   └─► logs/*.bin  │   .bin ────────┼─► (Lage + Höhe zur Scanmitte)
   LOCAL_POSITION_NED → T_POSE                    │   .pkl / USB ──┘          │
   DISTANCE_SENSOR   → T_RANGE                    │                           ▼
 ─────────────────────────────────────────────────┘          Mapper.process(winkel, distanz, sensoren)
                                                               │  Neigung ausgleichen → Matching
                                                               ▼
                                                     GridMap (2D)  /  LayerMap (2,5D)
```

---

## Schnellstart mit Flugcontroller

1. **Flugcontroller (Mission Planner / QGC):** Die serielle Schnittstelle zum Pi braucht
   `SERIALx_PROTOCOL = 2` (MAVLink2). Bei euch ist das **SERIAL6 mit `SERIAL6_BAUD = 460` (460800)**.
   Der Höhenmesser muss als Rangefinder 1 eingerichtet sein; unter *Rangefinder1* muss sich der Wert ändern.
2. **Pi, einmalig:**
   ```bash
   sudo apt install python3-serial
   python3 -m venv --system-site-packages ~/venv && ~/venv/bin/pip install pymavlink
   ```
3. **Dateien auf den Pi kopieren** (vom Laptop, Projektordner):
   ```bash
   scp pi/pi_streamer.py pi/tmini_plus.py protocol.py pi:~/
   ```
   (`pi` ist der Host-Eintrag in `~/.ssh/config`.)
4. **Pi starten:**
   ```bash
   ~/venv/bin/python3 pi_streamer.py --mav /dev/serial0
   ```
   Alle 5 s sollte etwa `ATT 20/s | Höhenmesser 20/s (0.85 m)` erscheinen.
5. **Laptop starten:**
   ```bash
   python lidar_map3d.py --pi 100.83.11.79      # 3D, Höhe kommt automatisch
   python lidar_map.py   --pi 100.83.11.79      # nur 2D
   ```

**Vor dem ersten Flug: Handtest.** Drohne ohne Propeller in die Hand nehmen, kippen, heben und senken.
Im Laptop-Fenster müssen Neigung und Höhe stimmen, und die Karte darf beim Kippen nicht verschmieren.

---

## Benutzung

### Installation am Laptop

```bash
pip install pyserial numpy scipy opencv-python matplotlib
pip install open3d            # optional, schönere 3D-Ansicht
```

### 2D-Karte (`lidar_map.py`)

```bash
python lidar_map.py                         # LiDAR per USB, COM-Port automatisch
python lidar_map.py --port COM5             # Port selbst angeben (Linux: /dev/ttyUSB0)
python lidar_map.py --record test.pkl       # zusätzlich Rohdaten aufzeichnen
python lidar_map.py --replay test.pkl       # Aufzeichnung abspielen (ohne LiDAR)
python lidar_map.py --replay test.pkl --speed 0   # so schnell wie möglich
python lidar_map.py --pi 100.83.11.79       # LiDAR hängt am Pi
python lidar_map.py --replay scan_XXXX.bin  # Pi-Aufzeichnung abspielen (mit Lage/Höhe)
```

| Option | Bedeutung |
|---|---|
| `--port` | serieller Port des LiDAR |
| `--record DATEI.pkl` | Rohdaten speichern (beim Beenden) |
| `--replay DATEI` | `.pkl` (Laptop) oder `.bin` (Pi) abspielen |
| `--pi IP` | Scans vom Pi holen (Tailscale-IP), Laptop meldet sich per HELLO an |
| `--udp PORT` | UDP-Port (Standard 5005); ohne `--pi`: warten, bis der Pi von sich aus sendet |
| `--speed` | Replay-Tempo (1 = Echtzeit, 0 = maximal) |
| `--save NAME` | Dateiname ohne Endung (Standard `karte`) |
| `--no-gui` | ohne Fenster |

**Tasten:** `s` speichern · `r` Karte zurücksetzen · `q`/`ESC` beenden (speichert auch)

**Statuszeile:**
- `match`: Anteil der Scanpunkte an bekannten Wänden
- `Hz`, `verworfen`
- `Neigung=…deg (verw. N)`: aktuelle Neigung und wie viele Scans wegen zu starker Neigung verworfen wurden. Erscheint nur mit FC-Daten.
- `Rueckstand`: Scans in der Queue, falls die Verarbeitung nicht hinterherkommt
- `UDP-Verlust`
- `VERLOREN`: mehr als 3 Scans hintereinander passten nicht

> Für die 2D-Karte reicht eine `.pkl`. Für 3D mit Neigungsausgleich die **`.bin` vom Pi**
> abspielen (`logs/` auf dem Pi): Eine `.pkl` von `lidar_map3d.py --pi --record` enthält zwar
> die Höhe, aber keine Roll/Pitch-Werte.

### 2,5D-Karte (`lidar_map3d.py`)

```bash
python lidar_map3d.py --pi 100.83.11.79     # mit FC: Höhe + Neigung automatisch
python lidar_map3d.py                       # LiDAR per USB, Höhe per Taste
python lidar_map3d.py --record raum3d.pkl   # Rohdaten + Höhen aufzeichnen
python lidar_map3d.py --replay raum3d.pkl --speed 0
python lidar_map3d.py --replay scan_XXXX.bin
```

Gleiche Optionen wie bei der 2D-Karte, Standardname ist `karte3d`.

**Tasten:** `+`/`-` Höhe ±10 cm · `p` Höhe = 0 · `v` 3D-Ansicht · `s` speichern · `q`/`ESC` Ende.
Die Höhentasten braucht man nur ohne Höhenmesser.

**Woher die Höhe kommt** (`sensor_height()`), in dieser Reihenfolge:

| Quelle | Bedingung | Höhe des LiDARs |
|---|---|---|
| Höhenmesser (`T_RANGE`) | Wert > 2 cm, höchstens 0,4 s alt | `distanz · cos(roll) · cos(pitch) + LIDAR_ABOVE_RANGEFINDER` |
| EKF (`T_POSE`, z) | höchstens 0,5 s alt | `-z` (NED) |
| Taste `+`/`-` | sonst | eingestellter Wert |
| `.pkl`-Replay | – | mitgespeicherte Höhe, sonst `START_HEIGHT` |

**Ohne Flugcontroller scannen:** auf einer Höhe langsam durch den Raum gehen, dann mit `+`/`-` die
neue Höhe einstellen und wiederholen. Beim Höhenwechsel den LiDAR möglichst an derselben Stelle
halten, damit die Position nicht verloren geht.

---

## Ausgabedateien

| Datei | Inhalt |
|---|---|
| `NAME.png` | Karte als Bild (weiß = frei, schwarz = Wand, grau = unbekannt), auf den erfassten Bereich zugeschnitten |
| `NAME.npy` | rohe Log-Odds-Werte (float32) desselben Ausschnitts |
| `NAME.yaml` | Metadaten im ROS-`map_server`-Format (Auflösung, Ursprung) |
| `NAME_trajectory.csv` | Pfad: `x, y, yaw, scan_idx` |
| `karte3d.ply` | 3D-Punktwolke (ASCII, nach Höhe eingefärbt), z. B. mit MeshLab / CloudCompare öffnen |
| `karte3d_2d.*` | die gemeinsame 2D-Karte des 3D-Laufs |
| `karte3d_schicht_0.20m.*` | eine 2D-Karte pro Schicht (`m` vor dem Wert = negative Höhe) |
| `*.pkl` | Laptop-Aufzeichnung: Liste von `(t, winkel, distanz, intensität[, höhe])` |
| `logs/scan_*.bin` (Pi) | Pi-Aufzeichnung: alle Nachrichten inkl. Lage und Höhenmesser |

Koordinaten: Start = (0, 0), Blickrichtung beim Start = +x, Einheit Meter. Alle diese Dateien
sind per `.gitignore` vom Repo ausgeschlossen.

---

## Wie es funktioniert (`lidar_map.py`)

### LiDAR-Treiber: `TMiniPlus`
- YDLIDAR-Protokoll, 230400 Baud. `A5 60` startet den Scan, `A5 65` stoppt ihn.
- Pakete beginnen mit `AA 55`, danach der Header `PH, CT, LSN, FSA, LSA, CS`.
- Ob ein Messpunkt 3 Byte (mit Intensität) oder 2 Byte groß ist, wird automatisch über
  die Prüfsumme erkannt. Passt die Prüfsumme 100-mal nie, wird ohne Prüfung gelesen.
- Ist in `CT` das Bit 0 gesetzt, beginnt eine neue Umdrehung. Ab 30 Punkten wird ein Scan ausgegeben.
- `find_port()` sucht nach typischen USB-Seriell-Chips (CP210x, CH340, ...).
- `pi/tmini_plus.py` ist derselbe Treiber in reinem Python (ohne numpy) für den Pi.

### Sensordaten zum Scan
Jeder Scan vom Pi bekommt ein dict `sensoren = {roll, pitch, yaw, range, ekf_z}`. Es wird von
`protocol.SensorState.at()` zum Zeitpunkt **Scanmitte** gebildet (Zeitstempel des Scans = Scan-Ende,
minus `SCAN_MID` = 0,08 s). Fehlende Werte sind `None`. Beim Abspielen einer `.bin` werden die Scans
0,3 s zurückgehalten, damit die zugehörigen Lage-Nachrichten schon gelesen sind.
Per USB oder aus einer `.pkl` gibt es keine Sensordaten; dann läuft alles ohne Neigungsausgleich.

### Verarbeitung pro Scan: `Mapper.process(winkel, distanz, sensoren=None)`
1. **Punkte:** Polarkoordinaten → x/y, nur Messungen zwischen `MIN_RANGE` und `MAX_RANGE`.
   Der Winkel wird gespiegelt (Uhrzeigersinn → mathematisch positiv, außer bei `FLIP`)
   und um `LIDAR_YAW_OFFSET_DEG` gedreht, damit 0° = Drohnen-Vorderseite.
2. **Neigungsausgleich** (nur mit Sensordaten und `TILT_COMP`):
   - Neigung = `acos(cos(roll) · cos(pitch))`. Liegt sie über `MAX_TILT_DEG` (15°), wird der Scan verworfen (`n_tilt`).
   - `level_points()` dreht jeden Punkt mit `R = Ry(pitch) · Rx(roll)` (ArduPilot-Konvention:
     roll rechts runter positiv, pitch Nase hoch positiv) in die Waagerechte. Ergebnis: die
     projizierten x/y-Werte plus die **Höhe jedes Punkts relativ zum LiDAR** (`zrel`).
   - **Bodentreffer:** Ist die Höhe bekannt, werden Punkte mit `höhe + zrel ≤ FLOOR_CLEARANCE` (10 cm)
     verworfen. Das sind geneigte Strahlen, die den Boden statt einer Wand treffen.
3. **Vorhersage:** neue Pose = alte Pose ⊕ letzte Bewegung pro Scan (`vel`).
4. **Referenz:** Wandzellen der Karte im Umkreis `MAX_RANGE + 1,5 m` werden zu einem
   KD-Baum, Normalen (PCA über 9 Nachbarn) und einem Wahrscheinlichkeitsfeld
   (Distanztransformation, Gauß mit `FIELD_SIGMA`).
5. **Entzerrung (`deskew`):** Während einer Umdrehung bewegt sich der LiDAR mit. Deshalb
   wird jeder Punkt anhand seines Zeitanteils auf die Pose am Scan-Ende umgerechnet.
6. **Korrelationssuche:** Alle Drehungen (Schritt 1°) und Verschiebungen innerhalb
   `SEARCH_YAW_DEG` / `SEARCH_XY_M` werden gegen das Wahrscheinlichkeitsfeld bewertet.
   Eine leichte Bevorzugung der Vorhersage verhindert Sprünge in symmetrischen Räumen.
7. **ICP-Feinabgleich:** Point-to-Line mit Huber-Gewichten, dreistufig
   (30 → 15 → 8 cm Suchradius). Ecken werden schwächer gewichtet als gerade Wände.
8. Die Schritte 5–7 laufen zweimal; im zweiten Durchgang wird mit der neu geschätzten Geschwindigkeit entzerrt.
9. **Qualität:** der Anteil der Punkte, die näher als 10 cm an einer Wand liegen. Gezählt wird nur
   in bereits bekannten Bereichen, damit neue Räume oder Fenster den Scan nicht verwerfen.
   - unter `RETRY_BELOW` (80 %): zusätzlich weiträumig suchen, das bessere Ergebnis nehmen
   - ab `MIN_INLIER_RATIO` (60 %): Scan wird übernommen und eingetragen; `last_zrel` enthält die Punkthöhen für die 3D-Karte
   - darunter: Scan wird verworfen, `lost` wird hochgezählt, und beim nächsten Scan wird weiträumig gesucht (±60°, ±1 m)

Solange die Karte weniger als `MIN_MAP_POINTS` Wandzellen hat, werden die Scans ohne Matching eingetragen.

### Karte: `GridMap`
- Occupancy Grid mit Log-Odds, 60 × 60 m bei 5 cm (1200 × 1200 Zellen), Start in der Mitte.
- Pro Scan werden die Zellen entlang jedes Strahls um `L_FREE` gesenkt (frei) und die Endpunkte
  um `L_OCC` erhöht (Wand). Die Werte werden auf `[L_MIN, L_MAX]` begrenzt.
- Ab `L_OCC_THRESH` gilt eine Zelle beim Matching als Wand.

### Spiegelfilter (experimentell, standardmäßig aus)
`MIRROR_FILTER = True` aktiviert ihn. Punkte, die deutlich *hinter* einer sicher erkannten, geraden
Wand (≥ 40 cm) liegen, gelten als Spiegelbild. Der Strahl wird dann an der Spiegelfläche gekürzt,
allerdings nur, wenn mindestens `MIRROR_MIN_RUN` benachbarte Strahlen betroffen sind.

### Wichtige Parameter

| Parameter | Standard | Wirkung |
|---|---|---|
| `RESOLUTION` | 0,05 m | Zellgröße |
| `MAP_SIZE_M` | 60 m | Kartengröße (Speicher wächst quadratisch) |
| `MIN_RANGE` / `MAX_RANGE` | 0,12 / 10 m | gültiger Messbereich |
| `FLIP` | False | auf True setzen, wenn die Karte spiegelverkehrt ist (LiDAR kopfüber) |
| `DESKEW` | True | Bewegungsverzerrung korrigieren |
| `TILT_COMP` | True | Neigungsausgleich mit Roll/Pitch vom FC |
| `MAX_TILT_DEG` | 15° | stärker geneigte Scans verwerfen (bei 10° gingen zu viele verloren und die Position riss ab) |
| `LIDAR_YAW_OFFSET_DEG` | 0° | Winkel zwischen 0°-Markierung des LiDARs und Drohnen-Vorderseite |
| `LIDAR_ABOVE_RANGEFINDER` | 0 m | wie viel höher der LiDAR als der Höhenmesser sitzt |
| `FLOOR_CLEARANCE` | 0,10 m | Punkte darunter gelten als Bodentreffer |
| `SEARCH_YAW_DEG` | (20, 60) | Drehsuche normal / nach Verlust |
| `SEARCH_XY_M` | (0,35, 1,0) | Verschiebungssuche normal / nach Verlust |
| `MIN_INLIER_RATIO` | 0,60 | darunter wird der Scan verworfen |
| `DRAW_HZ` | 5 | Bildwiederholrate des Fensters |

---

## 2,5D-Erweiterung (`lidar_map3d.py`)

- Die Position (x, y, yaw) kommt aus dem normalen 2D-`Mapper`. Er arbeitet auf **einer**
  gemeinsamen Karte über alle Höhen und mit den waagerecht projizierten Punkten.
- `LayerMap` hält zusätzlich pro Höhenschicht (`LAYER_H` = 10 cm) eine eigene `GridMap`.
- Jeder **akzeptierte** Scan wird eingetragen. Jeder Punkt bekommt seine echte Höhe
  `z = LiDAR-Höhe + zrel`. Ein geneigter Scan verteilt sich so auf **mehrere Schichten**.
  Ohne Neigungsdaten liegen alle Punkte auf der LiDAR-Höhe. Schichten mit weniger als 5 Punkten
  aus einem Scan werden für diesen Scan übersprungen.
- `LayerMap.points()` macht aus allen Zellen mit Log-Odds > `MIN_HITS_LOGODDS` (1,5) Punkte;
  z ist dabei die Schichtmitte.
- Die 3D-Ansicht nutzt open3d, falls installiert, sonst matplotlib (höchstens 60 000 Punkte, gleiche Achsenskalierung).

| Parameter | Standard | Wirkung |
|---|---|---|
| `LAYER_H` | 0,10 m | Dicke einer Schicht |
| `HEIGHT_STEP` | 0,10 m | Höhenänderung pro Tastendruck |
| `START_HEIGHT` | 0 m | Höhe beim Start (ohne Höhenmesser) |
| `MIN_HITS_LOGODDS` | 1,5 | ab hier gilt eine Zelle beim 3D-Export als belegt |

---

## Pi ↔ Laptop (`protocol.py`)

### Verbindungsaufbau
Der Laptop schickt jede Sekunde `DRHELLO1` an den Pi (Port 5005). Der Pi sendet an die Adresse des
letzten HELLO. Das funktioniert auch, wenn der Pi selbst keine Verbindungen aufbauen darf (Tailscale-Sharing).

### Nachrichten (Little Endian)

```
Header (11 Byte):  "DR" (2s) | typ (u8) | zeitstempel (f64, Pi-Zeit in s)

T_SCAN  (typ 1):   scan_id (u32) | teil (u8) | teile_gesamt (u8) | n (u16)
                   + n × winkel   (u16, 1/100 Grad, im Uhrzeigersinn)
                   + n × distanz  (u16, mm)
                   + n × intensität (u8)

T_POSE  (typ 2):   roll, pitch, yaw (f32, rad) | x, y, z (f32, m, NED; NaN = unbekannt)
                   (FC: ATTITUDE + LOCAL_POSITION_NED)

T_RANGE (typ 3):   distanz (f32, m) | ausrichtung (u8, MAVLink: 25 = nach unten)
                   (FC: DISTANCE_SENSOR, z. B. MTF-01P)
```

- Ein Scan wird in Teile zu höchstens 200 Punkten (~1 KB) zerlegt, damit die Pakete auch bei MTU 1280
  (Tailscale) nicht fragmentiert werden. Die Punkte bleiben in Aufnahmereihenfolge, was für die Entzerrung wichtig ist.
- `ScanAssembler` setzt die Teile wieder zusammen. Unvollständige Scans werden verworfen und als
  `lost` gezählt (sobald ein neuerer Scan fertig ist oder mehr als 3 Scans offen sind).
- `SensorState` puffert die FC-Nachrichten und liefert mit `at(t)` die Werte, die zeitlich am nächsten liegen:

  | Wert | Quelle | höchstens so weit weg von `t` |
  |---|---|---|
  | roll, pitch, yaw | `T_POSE` (letzte 60) | 0,2 s |
  | range | `T_RANGE` mit Ausrichtung 25 (letzte 30) | 0,4 s |
  | ekf_z | letzter `T_POSE` mit gültigem z | 0,5 s |

### Log-Datei (`.bin`)
Folge von `[länge (u16) | nachricht]`, also dieselben Nachrichten wie über UDP. `read_log()` liest sie,
und `lidar_map.bin_source()` macht daraus Scans mit Sensordaten.

### Funktionen

| Funktion | Zweck |
|---|---|
| `encode_scan(scan_id, t, winkel, dist, intens)` | Scan → Liste von Paketen |
| `encode_pose(t, roll, pitch, yaw, x, y, z)` | Lage/Position → Paket |
| `encode_range(t, distanz_m, ausrichtung)` | Höhenmesser → Paket |
| `decode(msg)` | Paket → `(typ, t, daten)` oder `None` |
| `ScanAssembler.add(t, daten)` | Teil hinzufügen → fertiger Scan oder `None` |
| `SensorState.add(typ, t, daten)` / `.at(t)` | FC-Werte puffern / zu einem Zeitpunkt abfragen |
| `read_log(path)` | Generator über alle Nachrichten einer `.bin`-Datei |

`protocol.py` muss auf Pi und Laptop **identisch** sein. Nach jeder Änderung wieder auf den Pi kopieren.

---

## Pi (`pi/pi_streamer.py`)

Liest den LiDAR (`tmini_plus.py`) und optional den Flugcontroller per MAVLink, schickt alles per
UDP an den Laptop und zeichnet es als `logs/scan_JJJJMMTT_HHMMSS.bin` auf. Der Pi hat kein Git: Die
Dateien werden im Repo bearbeitet und per `scp` kopiert (siehe Schnellstart).

```bash
python3 pi_streamer.py                                # nur LiDAR, wartet auf den Laptop
~/venv/bin/python3 pi_streamer.py --mav /dev/serial0  # + Flugcontroller (460800 Baud voreingestellt)
python3 pi_streamer.py --dest LAPTOP_IP:5005          # festes Ziel (Laptop: --udp 5005)
```

| Option | Standard | Bedeutung |
|---|---|---|
| `--lidar` | `/dev/ttyUSB0` | serieller Port des LiDAR |
| `--port` | 5005 | Port, auf dem der Pi auf HELLO wartet |
| `--dest IP:PORT` | – | festes Ziel statt auf den Laptop zu warten |
| `--log-dir` | `logs` | Ordner für `.bin`-Aufzeichnungen |
| `--no-log` | – | nicht aufzeichnen |
| `--mav` | – | Flugcontroller-Port (z. B. `/dev/serial0`) |
| `--mav-baud` | 460800 | muss zu `SERIALx_BAUD` passen (bei euch SERIAL6) |
| `--mav-rate` | 20 Hz | angeforderte Rate für ATTITUDE, LOCAL_POSITION_NED und DISTANCE_SENSOR |

**Was der FC-Teil macht:** Nach dem Heartbeat fordert er die drei Nachrichten per
`MAV_CMD_SET_MESSAGE_INTERVAL` an.
- Bei jeder `ATTITUDE` wird ein `T_POSE` gesendet. x/y/z sind NaN, wenn die letzte `LOCAL_POSITION_NED` älter als 0,5 s ist.
- Jede `DISTANCE_SENSOR` wird als `T_RANGE` gesendet (cm → m).
- Kommt 15 s lang kein Heartbeat, läuft nur der LiDAR weiter.

**Statuszeile alle 5 s:**
- Scans/s, Punkte pro Scan, Datenformat
- ob der Laptop verbunden ist
- Prüfsummen- und UDP-Fehler
- mit `--mav`: `ATT …/s | Höhenmesser …/s (letzter Wert)`

Meldet sich der Laptop 5 s lang nicht, pausiert das Senden, die Aufzeichnung läuft weiter.

---

## Grenzen

- **`/dev/serial0` kann nur ein Programm gleichzeitig benutzen.** Braucht noch etwas anderes den FC
  (z. B. YOLO-Daten zum FC), muss ein MAVLink-Router (z. B. `mavlink-router`) dazwischen.
- **LiDAR-Ausrichtung:** Der Neigungsausgleich nimmt an, dass die 0°-Markierung des LiDARs nach vorne zeigt,
  wie der FC. Sonst `LIDAR_YAW_OFFSET_DEG` anpassen, denn sonst werden Roll und Pitch auf die falsche Achse gerechnet.
- **Höhenmesser misst bis zum Boden direkt darunter.** Über einem Tisch springt die Höhe um die Tischhöhe.
- Scans über 15° Neigung gehen verloren. Bei stark wackelnden Flügen kann die Position abreißen.
- Bisher nur mit simulierter Drohne und simuliertem FC getestet, noch nicht mit dem echten MicoAir.

---

## Fehlersuche

| Problem | Ursache / Lösung |
|---|---|
| „LiDAR-Port nicht eindeutig“ | `--port COMx` angeben (Geräte-Manager) |
| „Warte auf Scans …“ (USB) | Dreht sich der LiDAR? Richtiger Port? Läuft kein anderes Programm auf dem Port? |
| „Warte auf Scans …“ (Pi) | Läuft `pi_streamer.py`? Stimmt die Tailscale-IP? Firewall/UDP 5005? |
| „Prüfsumme passt nie“ | falsche Baudrate oder anderes LiDAR-Modell |
| `[mav] kein Heartbeat` | falscher Port, `--mav-baud` ≠ `SERIALx_BAUD`, `SERIALx_PROTOCOL` ≠ 2, Port von anderem Programm belegt |
| `ModuleNotFoundError: pymavlink` | mit `~/venv/bin/python3` starten, nicht mit `python3` |
| `Höhenmesser 0/s` | Rangefinder im FC nicht eingerichtet, oder Ausrichtung nicht nach unten (25) |
| Neigung wird nicht angezeigt | keine `T_POSE` vom Pi (ohne `--mav` gestartet?) oder Abspielen einer `.pkl` |
| Karte verschmiert beim Kippen | `LIDAR_YAW_OFFSET_DEG` prüfen; Vorzeichen von Roll/Pitch im Handtest kontrollieren |
| Karte spiegelverkehrt | `FLIP = True` |
| viele `verworfen`, Karte verschmiert | langsamer bewegen, vor allem langsamer drehen; Aufnahme mit `--replay` erneut auswerten |
| `VERLOREN` | langsam zu einem bekannten Bereich zurückgehen, dann wird weiträumig gesucht |
| `Rueckstand` wächst | PC zu langsam: `MAP_SIZE_M` verkleinern oder aufnehmen und später abspielen |
| hoher `UDP-Verlust` | schlechtes WLAN; aufzeichnen und die `.bin` abspielen |
