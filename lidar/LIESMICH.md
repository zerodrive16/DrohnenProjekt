# LiDAR-Kartierung + Hindernisvermeidung: Pi -> Laptop / Flugcontroller

## Aufbau

    lidar/                Laptop (Bodenstation)
      main.py             EINZIGER Startpunkt: 2D, 3D, mit/ohne FC
      config.py           alle Einstellungen
      slam.py             Positionsschätzung (Scan-Matching, Neigungsausgleich, FC-Gierwinkel)
      maps.py             2D-Karte, 3D-Schichten, Speichern
      sources.py          Datenquellen: USB-LiDAR, UDP vom Pi, Aufzeichnungen
      viz.py              Fenster und 3D-Ansicht
      protocol.py         Funkprotokoll Pi <-> Laptop (identisch mit pi/protocol.py)
      tests/              Simulatoren + automatische Tests (nur lokal, nicht im Repo)
    pi/                   Raspberry Pi (nach ~/DrohnenProjekt)
      pi_streamer.py      LiDAR + FC-Daten an den Laptop, Hindernisdaten an den FC
      tmini_plus.py, protocol.py

Die alten Skripte lidar_map.py, lidar_map3d.py, lidar_map_fc.py und lidar_map3d_fc.py
sind durch main.py ersetzt und können gelöscht werden. Aus denselben Aufnahmen entstehen
Byte für Byte dieselben Karten.

## Änderungen (Umbau auf main.py + Hindernisvermeidung)

- Laptop-Code aufgeteilt in Module: `main.py` (Start), `config.py`, `slam.py`, `maps.py`,
  `sources.py`, `viz.py`. `main.py` ersetzt die alten Einzelskripte für 2D/3D mit/ohne FC.
- `pi/protocol.py` liegt jetzt direkt beim Pi-Code (Kopie von `protocol.py`), damit der
  Ordner `pi/` ohne weitere Dateien auf den Pi kopiert werden kann.
- `pi_streamer.py`:
  - Neue Hindernisvermeidung: Mit `--mav` werden pro Scan die kürzesten Abstände in
    72 Sektoren (je 5°) berechnet und mit 10 Hz als MAVLink `OBSTACLE_DISTANCE`
    (Frame BODY_FRD) an den Flugcontroller geschickt. Daten älter als 0,5 s werden nicht
    gesendet, damit der FC einen LiDAR-Ausfall bemerkt.
  - Neue Optionen: `--no-obstacle`, `--lidar-yaw-offset GRAD`, `--obstacle-min M`
    (Standard 0,25), `--obstacle-max M` (Standard 12).
  - MAVLink-Verbindung meldet sich als Begleitrechner (Komponente 191).
  - Statuszeile zeigt zusätzlich Hindernis-Senderate, nächstes Hindernis und Fehler.
  - Hilfetexte verweisen auf `main.py` statt `lidar_map.py`.

## Installation

Laptop:

    pip install pyserial numpy scipy opencv-python matplotlib pytest
    (optional, schönere 3D-Ansicht: pip install open3d)

Pi (einmalig): sudo apt install python3-serial   (pymavlink ist schon installiert)

## Start

Pi:

    python3 pi_streamer.py --mav /dev/serial0 --mav-baud 115200

Laptop (im Ordner lidar/, Pi-Hotspot "drone"):

    python main.py --pi 10.42.0.1                 2D-Karte
    python main.py --pi 10.42.0.1 --3d            3D-Karte
    python main.py --port COM5                    LiDAR direkt per USB am Laptop
    python main.py --replay scan_XXXX.bin         Aufzeichnung abspielen
    python main.py --replay scan_XXXX.bin --3d    ... als 3D-Karte
    python main.py --3d --auto 20                 ohne Höhenmesser: alle 20 s eine Ebene höher
    python main.py --3d --auto 20 --times 5       ... bei der 5. Erhöhung Ende (5 Ebenen, 100 s)

(über Tailscale statt 10.42.0.1 die 100.83.11.79)

Weitere Optionen: `python main.py --help`

- `--no-fc`  Gierwinkel des Flugcontrollers NICHT nutzen. Standard ist: nutzen, sobald der Pi
  FC-Daten schickt (pi_streamer.py mit --mav). Das hält die Karte bei schnellen Drehungen
  stabil. Ohne FC-Daten (LiDAR am Laptop, alte .pkl) macht die Option keinen Unterschied.
- `--save NAME`  Standard: karten/<datum_zeit>/karte (bzw. karte3d)
- `--record DATEI.pkl`  Rohdaten am Laptop aufzeichnen (am Pi wird ohnehin aufgezeichnet)

Tasten im Fenster: s = speichern, r = zurücksetzen, q/ESC = Ende (speichert auch)
3D zusätzlich: + / - = Höhe ±10 cm, p = Höhe 0, v = 3D-Ansicht

Die Statuszeile auf dem Pi zeigt alle 5 s z. B.:

    Scans 6.2/s | Laptop verbunden | ATT 20/s | Höhenmesser 20/s (0.85 m) | Hindernis->FC 10/s (nächstes 0.97 m)

## Tests

Im Ordner lidar/:

    python -m pytest tests -v -s                  alles (einige Minuten)
    python -m pytest tests/test_basics.py         nur die schnellen Tests (unter 1 s)

Geprüft wird:
- LiDAR-Parser und Funkprotokoll (Zerlegen/Zusammensetzen, verlorene Pakete, FC-Daten in .bin)
- Genauigkeit auf Simulationen: ruhiger Rundgang, schnelle Drehungen, geneigte Drohne,
  hektischer Anfängerflug (mit FC deutlich besser als ohne), 3D vom Laptop und im Flug
- Start-Fehler aus der Uni (Teilscan + Drehung beim Start) kommt nicht zurück
- Neigungsausgleich ist eingeschaltet
- Aufzeichnungen lassen sich wieder abspielen und ergeben dieselbe Karte

Nach jeder Änderung an config.py oder slam.py einmal laufen lassen.

## Hindernisvermeidung (LiDAR -> Flugcontroller)

pi_streamer.py schickt mit --mav automatisch die kürzesten Abstände in 72 Richtungen
(je 5°, bis 12 m) als MAVLink OBSTACLE_DISTANCE mit 10 Hz an den FC.
Ausschalten: --no-obstacle

### Einmalig im Flugcontroller (Mission Planner -> Config -> Full Parameter List)

    PRX1_TYPE    = 2    (MAVLink)
    AVOID_ENABLE = 7    (alle Quellen inkl. Proximity)
    AVOID_BEHAVE = 1    (anhalten; 0 = am Hindernis entlang gleiten)
    AVOID_MARGIN = 1.0  (m Sicherheitsabstand, am Anfang lieber größer)

Danach FC neu starten.

Hinweise:
- Die Vermeidung wirkt in Loiter und AltHold, NICHT in Stabilize/Acro.
  Loiter braucht eine Positionsschätzung (ohne GPS: optischer Fluss bzw. später SLAM-Position).
- Ist PRX1_TYPE=2 gesetzt und pi_streamer.py läuft nicht, meldet der FC
  "PreArm: PRX1: No Data" und lässt sich nicht scharf schalten. Das ist gewollt.
- Kontrolle: Mission Planner -> Strg+F -> "Proximity" zeigt das Radarbild.
  Strg+F -> "MAVLink Inspector" -> OBSTACLE_DISTANCE muss ~10/s ankommen.

### Einbau des LiDARs

- 0°-Markierung des LiDARs nach vorne (Drohnenvorderseite). Wenn nicht möglich:
  Winkel messen und --lidar-yaw-offset GRAD angeben (im Uhrzeigersinn).
  Test: Hand vor die Drohne halten -> im Radarbild muss das Hindernis vorne erscheinen.
- Alles unter 25 cm wird ignoriert (eigene Arme/Propeller). Bei großem Propellerschutz:
  --obstacle-min 0.35
- Der LiDAR sieht nur seine Ebene: Tischplatten, Lampen, Decke darüber/darunter nicht.

## Aufzeichnungen

Jede Fahrt liegt auf dem Pi unter logs/scan_....bin.
Abspielen am Laptop: python main.py --replay scan_....bin (mit --3d als 3D-Karte)
