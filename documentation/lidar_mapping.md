# LiDAR-Kartierung (2D und 2,5D)

Drei Module erzeugen aus dem **YDLIDAR T-mini Plus** eine Karte, ohne GPS und ohne
Flugcontroller. Die Position wird allein aus den Scans geschätzt (Scan-Matching / SLAM).

| Datei | Aufgabe |
|---|---|
| `lidar_map.py` | 2D-Karte: LiDAR-Treiber, Datenquellen, Scan-Matching, Occupancy Grid, Anzeige |
| `lidar_map3d.py` | 2,5D-Karte: eine 2D-Karte pro Höhenschicht → 3D-Punktwolke (`.ply`) |
| `protocol.py` | Nachrichtenformat zwischen Pi (`pi_streamer.py`) und Laptop, auch für Log-Dateien (`.bin`) |

```
                ┌─────────────── Datenquelle ───────────────┐
 LiDAR (USB) ──►│ TMiniPlus.scans()                         │
 Pi (UDP)    ──►│ udp_source()   ◄── protocol.decode()      │──► Queue ──► Mapper.process()
 .pkl / .bin ──►│ replay_source()                           │                 │
                └───────────────────────────────────────────┘                 ▼
                                                              GridMap (2D)  /  LayerMap (2,5D)
                                                                     │
                                                         .png / .npy / .yaml / .csv / .ply
```

Die Datenquelle läuft in einem eigenen Thread und legt jede Umdrehung als
`(winkel_grad, distanz_mm[, höhe])` in eine Queue. Der Hauptthread verarbeitet die
Scans und zeichnet das Fenster.

---

## Installation

```bash
pip install pyserial numpy scipy opencv-python matplotlib
pip install open3d            # optional, schönere 3D-Ansicht
```

`protocol.py` braucht nur die Standardbibliothek und läuft deshalb auch auf dem Pi ohne numpy.

---

## Benutzung

### 2D-Karte (`lidar_map.py`)

```bash
python lidar_map.py                         # COM-Port automatisch suchen
python lidar_map.py --port COM5             # Port selbst angeben (Linux: /dev/ttyUSB0)
python lidar_map.py --record test.pkl       # zusätzlich Rohdaten aufzeichnen
python lidar_map.py --replay test.pkl       # Aufzeichnung abspielen (ohne LiDAR)
python lidar_map.py --replay test.pkl --speed 0   # so schnell wie möglich
python lidar_map.py --pi 100.83.11.79       # LiDAR hängt am Pi (pi_streamer.py läuft dort)
python lidar_map.py --replay scan_XXXX.bin  # Pi-Aufzeichnung abspielen
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

**Statuszeile:** `match` = Anteil der Scanpunkte an bekannten Wänden · Hz · `verworfen` ·
`Rueckstand` (Scans in der Queue, falls die Verarbeitung nicht hinterherkommt) ·
`UDP-Verlust` · `VERLOREN`, wenn mehr als 3 Scans hintereinander nicht passen.

> **Tipp:** Live mit `--record` aufnehmen und danach mit `--replay` in Ruhe auswerten.
> Das Ergebnis ist gleich, aber ohne Zeitdruck, und man kann Parameter ausprobieren.

### 2,5D-Karte (`lidar_map3d.py`)

```bash
python lidar_map3d.py                       # LiDAR automatisch suchen
python lidar_map3d.py --record raum3d.pkl   # Rohdaten + Höhen aufzeichnen
python lidar_map3d.py --replay raum3d.pkl --speed 0
python lidar_map3d.py --pi 100.83.11.79     # Höhe kommt vom Flugcontroller, falls gesendet
```

Gleiche Optionen wie oben, Standardname ist `karte3d`.

**Tasten:** `+`/`-` Höhe ±10 cm · `p` Höhe = 0 · `v` 3D-Ansicht · `s` speichern · `q`/`ESC` Ende

**So wird gescannt:**
1. Auf der Starthöhe (z. B. Tischhöhe) einmal langsam durch den Raum gehen.
2. Höhe ändern und mit `+`/`-` die neue Höhe einstellen, dann wieder durch den Raum.
3. Für weitere Höhen wiederholen. Beim Höhenwechsel den LiDAR möglichst an derselben
   Stelle halten, damit die Position nicht verloren geht.

**Woher die Höhe kommt:**

| Quelle | Höhe |
|---|---|
| LiDAR am Laptop | Taste `+`/`-` |
| Pi live (`--pi`) | `-z` aus `T_POSE` (NED), sonst Taste |
| Replay `.bin` | `-z` aus `T_POSE`, sonst Taste |
| Replay `.pkl` | mitgespeicherte Höhe, sonst `START_HEIGHT` |

Später soll die Höhe vom Höhenmesser der Drohne (MTF-01P) kommen.

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

Koordinaten: Start = (0, 0), Blickrichtung beim Start = +x, Einheit Meter.

---

## Wie es funktioniert (`lidar_map.py`)

### LiDAR-Treiber: `TMiniPlus`
- YDLIDAR-Protokoll, 230400 Baud. `A5 60` startet den Scan, `A5 65` stoppt ihn.
- Pakete beginnen mit `AA 55`, danach der Header `PH, CT, LSN, FSA, LSA, CS`.
- Ob ein Messpunkt 3 Byte (mit Intensität) oder 2 Byte groß ist, wird automatisch über
  die Prüfsumme erkannt. Passt die Prüfsumme 100-mal nie, wird ohne Prüfung gelesen.
- Ist in `CT` das Bit 0 gesetzt, beginnt eine neue Umdrehung. Ab 30 Punkten wird ein Scan ausgegeben.
- `find_port()` sucht nach typischen USB-Seriell-Chips (CP210x, CH340, ...).

### Verarbeitung pro Scan: `Mapper.process()`
1. **Punkte:** Polarkoordinaten → x/y, nur Messungen zwischen `MIN_RANGE` und `MAX_RANGE`.
   Der Winkel wird gespiegelt (Uhrzeigersinn → mathematisch positiv), außer bei `FLIP`.
2. **Vorhersage:** neue Pose = alte Pose ⊕ letzte Bewegung pro Scan (`vel`).
3. **Referenz:** Wandzellen der Karte im Umkreis `MAX_RANGE + 1,5 m` werden zu einem
   KD-Baum, Normalen (PCA über 9 Nachbarn) und einem Wahrscheinlichkeitsfeld
   (Distanztransformation, Gauß mit `FIELD_SIGMA`).
4. **Entzerrung (`deskew`):** Während einer Umdrehung bewegt sich der LiDAR mit. Deshalb
   wird jeder Punkt anhand seines Zeitanteils auf die Pose am Scan-Ende umgerechnet.
5. **Korrelationssuche:** Alle Drehungen (Schritt 1°) und Verschiebungen innerhalb
   `SEARCH_YAW_DEG` / `SEARCH_XY_M` werden gegen das Wahrscheinlichkeitsfeld bewertet.
   Eine leichte Bevorzugung der Vorhersage verhindert Sprünge in symmetrischen Räumen.
   Das findet die Lage auch bei schnellen Drehungen.
6. **ICP-Feinabgleich:** Point-to-Line mit Huber-Gewichten, dreistufig
   (30 → 15 → 8 cm Suchradius). Ecken werden schwächer gewichtet als gerade Wände.
7. Die Schritte 4–6 laufen zweimal; im zweiten Durchgang wird mit der neu geschätzten Geschwindigkeit entzerrt.
8. **Qualität:** der Anteil der Punkte, die näher als 10 cm an einer Wand liegen. Gezählt wird
   nur in bereits bekannten Bereichen, damit neue Räume oder Fenster den Scan nicht verwerfen.
   - unter `RETRY_BELOW` (80 %): zusätzlich weiträumig suchen, das bessere Ergebnis nehmen
   - ab `MIN_INLIER_RATIO` (60 %): Scan wird übernommen und in die Karte eingetragen
   - darunter: Scan wird verworfen, `lost` wird hochgezählt, und beim nächsten Scan wird weiträumig gesucht
     (±60°, ±1 m)

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
| `SEARCH_YAW_DEG` | (20, 60) | Drehsuche normal / nach Verlust |
| `SEARCH_XY_M` | (0,35, 1,0) | Verschiebungssuche normal / nach Verlust |
| `MIN_INLIER_RATIO` | 0,60 | darunter wird der Scan verworfen |
| `DRAW_HZ` | 5 | Bildwiederholrate des Fensters |

---

## 2,5D-Erweiterung (`lidar_map3d.py`)

Der LiDAR scannt nur waagerecht. Für eine 3D-Karte wird auf mehreren Höhen gescannt:

- Die Position (x, y, yaw) kommt weiterhin aus dem normalen 2D-`Mapper`. Er arbeitet auf **einer**
  gemeinsamen Karte über alle Höhen.
- `LayerMap` hält zusätzlich pro Höhenschicht (`LAYER_H` = 10 cm) eine eigene `GridMap`.
  Jeder **akzeptierte** Scan wird in die Schicht seiner aktuellen Höhe eingetragen.
- `LayerMap.points()` macht aus allen Zellen mit Log-Odds > `MIN_HITS_LOGODDS` (1,5) Punkte;
  z ist dabei die Schichtmitte.
- Die 3D-Ansicht nutzt open3d, falls installiert, sonst matplotlib (höchstens 60 000 Punkte, gleiche Achsenskalierung).

Einschränkung: Neigung (roll/pitch) wird noch nicht berücksichtigt. Der LiDAR muss also möglichst
waagerecht gehalten werden. Wenn sich der Raum mit der Höhe stark ändert (z. B. Tische vs. Wände),
kann das Matching auf der gemeinsamen 2D-Karte schlechter werden.

---

## Pi ↔ Laptop (`protocol.py`)

Auf dem Pi liest `pi_streamer.py` den LiDAR (und optional den Flugcontroller per MAVLink)
und schickt die Daten per UDP über Tailscale an den Laptop. Zusätzlich kann er sie als `.bin` aufzeichnen.

### Verbindungsaufbau
Der Laptop schickt jede Sekunde `DRHELLO1` an den Pi (Port 5005). Der Pi sendet an die Adresse des
letzten HELLO. Das funktioniert auch, wenn der Pi selbst keine Verbindungen aufbauen darf (Tailscale-Sharing).

### Nachrichten (Little Endian)

```
Header (11 Byte):  "DR" (2s) | typ (u8) | zeitstempel (f64, Pi-Zeit in s)

T_SCAN (typ 1):    scan_id (u32) | teil (u8) | teile_gesamt (u8) | n (u16)
                   + n × winkel   (u16, 1/100 Grad, im Uhrzeigersinn)
                   + n × distanz  (u16, mm)
                   + n × intensität (u8)

T_POSE (typ 2):    roll, pitch, yaw (f32, rad) | x, y, z (f32, m, NED; NaN = unbekannt)
```

- Ein Scan wird in Teile zu höchstens 200 Punkten (~1 KB) zerlegt, damit die Pakete auch bei MTU 1280
  (Tailscale) nicht fragmentiert werden. Die Punkte bleiben in Aufnahmereihenfolge, was für die Entzerrung wichtig ist.
- `ScanAssembler` setzt die Teile wieder zusammen. Unvollständige Scans werden verworfen und als
  `lost` gezählt (sobald ein neuerer Scan fertig ist oder mehr als 3 Scans offen sind).
- `T_POSE` wird nicht in den Scan eingebaut. Die Empfänger merken sich die **letzte** Pose und hängen sie an den nächsten Scan.

### Log-Datei (`.bin`)
Folge von `[länge (u16) | nachricht]`, also dieselben Nachrichten wie über UDP. `read_log()` liest sie,
und `lidar_map.bin_source()` macht daraus Scans.

### Funktionen

| Funktion | Zweck |
|---|---|
| `encode_scan(scan_id, t, winkel, dist, intens)` | Scan → Liste von Paketen |
| `encode_pose(t, roll, pitch, yaw, x, y, z)` | Pose → Paket |
| `decode(msg)` | Paket → `(typ, t, daten)` oder `None` |
| `ScanAssembler.add(t, daten)` | Teil hinzufügen → fertiger Scan oder `None` |
| `read_log(path)` | Generator über alle Nachrichten einer `.bin`-Datei |

`protocol.py` muss auf Pi und Laptop **identisch** sein.

---

## Fehlersuche

| Problem | Ursache / Lösung |
|---|---|
| „LiDAR-Port nicht eindeutig“ | `--port COMx` angeben (Geräte-Manager) |
| „Warte auf Scans …“ (USB) | Dreht sich der LiDAR? Richtiger Port? Läuft kein anderes Programm auf dem Port? |
| „Warte auf Scans …“ (Pi) | Läuft `pi_streamer.py`? Stimmt die Tailscale-IP? Firewall/UDP 5005? |
| „Prüfsumme passt nie“ | falsche Baudrate oder anderes LiDAR-Modell |
| Karte spiegelverkehrt | `FLIP = True` |
| viele `verworfen`, Karte verschmiert | langsamer bewegen, vor allem langsamer drehen; Aufnahme mit `--replay` erneut auswerten |
| `VERLOREN` | langsam zu einem bekannten Bereich zurückgehen, dann wird weiträumig gesucht |
| `Rueckstand` wächst | PC zu langsam: `MAP_SIZE_M` verkleinern oder aufnehmen und später abspielen |
| hoher `UDP-Verlust` | schlechtes WLAN; aufzeichnen und die `.bin` abspielen |
