# Tutorial Aufgabe 2: Autonomer Flug ohne GPS

Stand: 28.10.2026 · Entwurf

Aufgabe 2 ist erfüllt, wenn die Drohne **ohne GPS stabil fliegt**, **Befehle vom Laptop selbstständig ausführt** (Guided-Modus) und der Pilot **jederzeit übernehmen oder per Not-Aus stoppen** kann. Die Flugmodi Stabilize, AltHold und Loiter (Stufen 1 bis 3) sind bereits getestet. Dieses Tutorial führt von dort zu Stufe 4, dem autonomen Flug.

> **Legende:** Angaben mit *(unsicher)* sind nicht geprüft und müssen vor der Umsetzung in der [ArduPilot-Dokumentation](https://ardupilot.org/copter/) oder im Handbuch des Geräts kontrolliert werden. Parameterwerte ohne Markierung stammen aus den Drohnendetails des Projekts.

## Inhalt

0. [Überblick](#0-überblick)
1. [Indoor-Grundlage prüfen](#1-indoor-grundlage-prüfen)
2. [Fernsteuerung einrichten](#2-fernsteuerung-einrichten-gx12-edgetx)
3. [Flight Controller einrichten](#3-flight-controller-einrichten-qgroundcontrol)
4. [Raspberry Pi einrichten](#4-raspberry-pi-einrichten)
5. [Laptop einrichten](#5-laptop-einrichten)
6. [EKF-Origin setzen](#6-ekf-origin-setzen)
7. [Testen in Stufen](#7-testen-in-stufen)
8. [Ausblick: Hinderniserkennung mit MT-06](#8-ausblick-hinderniserkennung-mit-mt-06)
9. [Checkliste, Fehlersuche, offene Fragen](#9-checkliste-fehlersuche-offene-fragen)

---

## 0. Überblick

### Anforderungen aus der Aufgabenstellung

- Autopilot ArduPilot integrieren und konfigurieren
- Kommunikation Autopilot ↔ Einplatinencomputer ↔ Bodenstation über MAVLink
- Stabiler und sicherer Flug ohne GPS
- Sensoren für Entfernung, optischen Fluss, Hinderniserkennung und Positionsschätzung
- Jederzeit manuelle Übernahme und Not-Aus
- Sicherer Zustand bei Funkverlust (aus der Projektbeschreibung)

### Wo wird was gemacht

| Ort | Werkzeug | Schritte |
|---|---|---|
| Fernsteuerung Radiomaster GX12 | EdgeTX-Menü am Sender | 2 |
| Flight Controller MicoAir743 V2 | QGroundControl per USB | 1, 3, 6 |
| Raspberry Pi Zero 2 WH | Raspberry Pi Imager, SSH-Terminal | 4 |
| Laptop | QGroundControl, Python | 5, 6, 7 |

### Datenweg im autonomen Flug

```
Laptop  --WLAN/UDP-->  Raspberry Pi  --UART-->  Flight Controller  -->  Motoren
  (Ziele, Befehle)        (leitet weiter)          (fliegt, sichert ab)

Fernsteuerung  --ELRS-Funk-->  Empfänger  --UART-->  Flight Controller
  (Pilot: Übernahme, Not-Aus – unabhängig von Pi und WLAN)
```

### Sicherheitsregeln für alle Tests

- Neue Einstellungen immer zuerst **ohne Propeller** testen.
- Der Pilot hält bei jedem Flug die Fernsteuerung und kann sofort in **Loiter** schalten oder den **Not-Aus** auslösen.
- Flüge mit Propellern nur in einem abgesicherten Raum, möglichst mit Propellerschutz oder Netz.
- Jede Parameteränderung notieren (alter Wert, neuer Wert, Datum). Das hilft bei Fehlern und für die Doku.
- Vor größeren Änderungen in QGroundControl die Parameter sichern: **Vehicle Setup → Parameters → Tools → Save to file**.

---

## 1. Indoor-Grundlage prüfen

**Wo:** QGroundControl am Laptop, Drohne per USB oder Funk verbunden. **Ziel:** Die Drohne hält drinnen in Loiter ihre Position. Ohne diese Grundlage funktioniert Guided nicht.

1. **EKF-Source-Set 3 aktivieren.** Mit dem Schalter auf Kanal 8 (`RC8_OPTION = 90`) auf Source-Set 3 schalten: optischer Fluss + Abstandssensor, Höhe aus dem MTF-01P.
   - Prüfen: In QGroundControl erscheint bei der Umschaltung eine Meldung zum Source-Set *(unsicher, genauer Wortlaut)*.
   - Einstellungen laut Drohnendetails: `EK3_SRC3_POSXY = 0`, `EK3_SRC3_POSZ = 2`, `EK3_SRC3_VELXY = 5`, `EK3_SRC3_VELZ = 0`, `EK3_SRC3_YAW = 1`.
2. **Flusssensor prüfen.** Unter **Analyze Tools → MAVLink Inspector** müssen die Nachrichten `OPTICAL_FLOW` und `DISTANCE_SENSOR` mit plausiblen Werten erscheinen. Die Drohne von Hand anheben: Der Abstand muss steigen.
3. **Kalibrierung des Flusssensors.** Falls noch nicht geschehen, nach der Anleitung [Optical Flow Sensor Setup](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html) kalibrieren (Parameter `FLOW_FXSCALER`, `FLOW_FYSCALER`) *(unsicher, ob für den MTF-01P nötig)*.
4. **Kompass drinnen prüfen.** Drohne im Testraum langsam einmal um die eigene Achse drehen. Die Richtungsanzeige in QGroundControl muss gleichmäßig mitlaufen und darf nicht springen. Stahlträger und Leitungen in Gebäuden stören den Kompass häufig.
5. **Loiter-Schwebetest.** In Loiter auf etwa 1 m steigen, Knüppel loslassen. Die Drohne sollte 30 Sekunden ohne Eingriff nahezu auf der Stelle stehen.
   - Driftet sie stark: Boden mit wenig Struktur (einfarbig, glänzend) kann den Flusssensor stören. Testweise Klebeband oder Teppich mit Muster auslegen.

**Erledigt, wenn:** Die Drohne in Loiter mit Source-Set 3 stabil schwebt und der Kompass drinnen ruhig ist.

---

## 2. Fernsteuerung einrichten (GX12, EdgeTX)

**Wo:** Menü der Radiomaster GX12, Modell der Drohne auswählen. **Ziel:** Feste Schalter für Flugmodus, Not-Aus und EKF-Umschaltung.

| Kanal | Schalter (Vorschlag) | Funktion |
|---|---|---|
| CH5 | 3-Stufen-Schalter, z. B. SA | Flugmodus: Loiter / AltHold / Guided |
| CH7 | 2-Stufen-Schalter, z. B. SF | Not-Aus (Motoren sofort aus) |
| CH8 | bereits belegt | EKF-Source-Set |

Die Schalterbezeichnungen der GX12 bitte am Gerät prüfen *(unsicher)*.

1. **Model → Mixes** öffnen.
2. **CH5** auswählen → Source: 3-Stufen-Schalter, Weight 100 %.
3. **CH7** auswählen → Source: 2-Stufen-Schalter, Weight 100 %. Einen Schalter wählen, der nicht versehentlich umgelegt wird, aber schnell erreichbar ist.
4. **CH8** kontrollieren, dass die bestehende Belegung erhalten bleibt.
5. **Kontrolle:** In QGroundControl unter **Vehicle Setup → Radio** jeden Schalter umlegen. Der passende Kanalbalken muss springen.

**Erledigt, wenn:** Alle drei Schalter in QGroundControl am richtigen Kanal ankommen.

---

## 3. Flight Controller einrichten (QGroundControl)

**Wo:** QGroundControl, Drohne per USB verbunden, **Propeller ab**. Parameter unter **Vehicle Setup → Parameters** suchen und setzen.

### 3.1 Flugmodi

1. `FLTMODE_CH = 5`
2. Unter **Vehicle Setup → Flight Modes** den 3-Stufen-Schalter durchschalten. QGroundControl hebt den aktiven Platz hervor. Für jede der drei Stellungen den Modus wählen:
   - Stellung oben: **Loiter** (Standard und Übernahme durch den Pilot)
   - Stellung Mitte: **AltHold** (Rückfall, falls die Positionsschätzung versagt)
   - Stellung unten: **Guided** (Laptop fliegt)

> **Warum Guided auf den Schalter?** ArduPilot wechselt den Modus nur, wenn sich die **Stellung** des Schalters ändert *(unsicher, in der Doku prüfen)*. Mit Guided auf dem Schalter ist die Übernahme eindeutig: Schalter von Guided zurück auf Loiter = Pilot hat sofort die Kontrolle.

### 3.2 Not-Aus

| Parameter | Wert | Hinweis |
|---|---|---|
| `RC7_OPTION` | Motor Emergency Stop | Option in der Auswahlliste von QGroundControl wählen, nicht die Nummer raten |

Test **ohne Propeller**: armen, Not-Aus-Schalter umlegen → Motoren müssen sofort stoppen.

### 3.3 Verbindung zum Raspberry Pi

Belegt sind SERIAL2 (Videosender), SERIAL4 (MTF-01P), SERIAL6 (ELRS-Empfänger) und ein Port für das GPS. Den freien Port aus der Pinbelegung des MicoAir743 V2 bestimmen *(offen)*. Für diesen Port (x = Nummer):

| Parameter | Wert | Bedeutung |
|---|---|---|
| `SERIALx_PROTOCOL` | 2 | MAVLink 2 |
| `SERIALx_BAUD` | 115 | 115200 Baud (für den Anfang, später ggf. 921) |

### 3.4 Failsafes

Ohne GPS kann die Drohne nicht sicher zum Start zurückfliegen. Deshalb ist **Landen** die passende Reaktion. Die genauen Werte für "Land" in der Auswahlliste von QGroundControl wählen *(unsicher)*.

| Parameter | Auslöser | Reaktion |
|---|---|---|
| `FS_THR_ENABLE` | Funkverlust zur Fernsteuerung | Land |
| `FS_GCS_ENABLE` | Verbindung zum Laptop bricht ab (WLAN) | Land |
| `FS_EKF_ACTION` | Positionsschätzung unzuverlässig | Land *(unsicher, Standardwert prüfen)* |
| `BATT_FS_LOW_ACT` | Akku niedrig | Land |

Hinweis: `FS_GCS_ENABLE` löst nur aus, wenn vorher eine Bodenstation verbunden war *(unsicher)*. Für die Tests ist das gewollt.

Nach allen Änderungen den Flight Controller neu starten (Stromlos machen oder **Tools → Reboot Vehicle**).

**Erledigt, wenn:** Flugmodi und Not-Aus ohne Propeller funktionieren und die Failsafes gesetzt sind.

---

## 4. Raspberry Pi einrichten

**Wo:** zuerst am Laptop (SD-Karte beschreiben), dann per SSH auf dem Pi.

### 4.1 Betriebssystem

1. Mit dem [Raspberry Pi Imager](https://www.raspberrypi.com/software/) **Raspberry Pi OS Lite (64-bit)** auf die 32-GB-microSD schreiben.
2. Vor dem Schreiben in den Einstellungen setzen:
   - Hostname: `drohne-pi`
   - Benutzer und Passwort
   - WLAN: Netzname und Passwort des Test-Netzes. Der Pi Zero 2 WH kann nur **2,4 GHz** *(unsicher, Datenblatt prüfen)*.
   - SSH aktivieren
3. SD-Karte einstecken, Pi starten, vom Laptop verbinden:
   ```
   ssh <benutzer>@drohne-pi.local
   ```

### 4.2 Serielle Schnittstelle

1. `sudo raspi-config` → **Interface Options → Serial Port**
   - Login-Shell über seriell: **Nein**
   - Serielle Schnittstelle aktivieren: **Ja**
2. Für eine stabile Verbindung den vollwertigen UART nutzen. Dazu in `/boot/firmware/config.txt` am Ende ergänzen *(unsicher, Pfad und Overlay je nach OS-Version prüfen)*:
   ```
   dtoverlay=disable-bt
   ```
3. Neu starten: `sudo reboot`. Danach ist die Schnittstelle als `/dev/serial0` verfügbar.

### 4.3 Verkabelung

Das Kabel aus dem Koffer verwenden. **TX und RX werden über Kreuz verbunden:**

| Raspberry Pi | | Flight Controller (freier UART) |
|---|---|---|
| Pin 8 (GPIO14, TX) | → | RX |
| Pin 10 (GPIO15, RX) | ← | TX |
| Pin 6 (GND) | — | GND |

- Beide Seiten arbeiten mit 3,3-V-Signalen *(unsicher, für den MicoAir743 V2 prüfen)*.
- **Stromversorgung:** Ob der 5-V-BEC des Flight Controllers (3 A, mit anderen Geräten geteilt) den Pi zuverlässig versorgt, ist offen. Für die ersten Tests den Pi per USB-Kabel aus der Powerbank versorgen.

### 4.4 MAVProxy installieren und testen

```
sudo apt update
sudo apt install -y python3-pip python3-venv
python3 -m venv ~/mav
~/mav/bin/pip install mavproxy
```

Verbindung zum Flight Controller testen:
```
~/mav/bin/mavproxy.py --master=/dev/serial0 --baudrate 115200
```
Erscheint der aktuelle Flugmodus (z. B. `LOITER>`), steht die Verbindung. Mit `Ctrl+C` beenden.

### 4.5 Weiterleitung an den Laptop

IP-Adresse des Laptops herausfinden (Windows: `ipconfig`, Linux: `ip a`). Dann auf dem Pi:
```
~/mav/bin/mavproxy.py --master=/dev/serial0 --baudrate 115200 \
  --out=udp:<Laptop-IP>:14550 --out=udp:<Laptop-IP>:14551 --daemon
```
- Port **14550** für QGroundControl
- Port **14551** für das Testskript (zwei Programme können nicht denselben Port nutzen)

**Optional: automatisch beim Start.** Datei `/etc/systemd/system/mavproxy.service` anlegen:
```
[Unit]
Description=MAVLink-Weiterleitung FC -> Laptop
After=network-online.target

[Service]
User=<benutzer>
ExecStart=/home/<benutzer>/mav/bin/mavproxy.py --master=/dev/serial0 --baudrate 115200 --out=udp:<Laptop-IP>:14550 --out=udp:<Laptop-IP>:14551 --daemon --non-interactive
Restart=always

[Install]
WantedBy=multi-user.target
```
Aktivieren: `sudo systemctl enable --now mavproxy`

Damit das klappt, braucht der Laptop im Test-Netz eine feste IP-Adresse (im Router einstellen).

**Erledigt, wenn:** MAVProxy auf dem Pi den Flugmodus des Flight Controllers anzeigt.

---

## 5. Laptop einrichten

1. **Netz:** Laptop und Pi im selben WLAN. Ein eigener Router oder Handy-Hotspot ist meist einfacher als das Hochschul-WLAN, das Geräte oft voneinander abschottet *(unsicher, beim Rechenzentrum erfragen)*.
2. **QGroundControl starten.** Es lauscht automatisch auf UDP-Port 14550. Die Drohne muss jetzt **ohne USB-Kabel über WLAN** erscheinen.
3. **Python-Paket installieren:**
   ```
   pip install pymavlink
   ```
4. **Windows-Firewall:** Beim ersten Start fragt Windows evtl. nach Netzwerkzugriff für Python und QGroundControl → für private Netze erlauben.
5. **Testskript** `guided_test.py` in einen Projektordner legen.

**Erledigt, wenn:** QGroundControl die Drohne über WLAN anzeigt und `python guided_test.py --help` ohne Fehler läuft.

---

## 6. EKF-Origin setzen

**Warum:** Ohne GPS kennt der EKF keinen Ursprung für Koordinaten. Loiter funktioniert trotzdem, aber ArduPilot lehnt Guided-Befehle mit Positionen nach meinem Wissen ab, solange kein Ursprung gesetzt ist *(unsicher, in der Doku zur [Non-GPS-Navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html) prüfen)*.

**Wann:** nach jedem Neustart des Flight Controllers, wenn die Drohne am Boden steht und Source-Set 3 aktiv ist.

**Wie:** mit dem Testskript. Die Koordinate ist beliebig, sie muss nur fest bleiben. Sinnvoll ist die ungefähre Lage des Testgebäudes:
```
python guided_test.py --connect udpin:0.0.0.0:14551 --set-origin <Breitengrad> <Längengrad> --dry-run
```
Alternativ geht es in Mission Planner per Rechtsklick auf die Karte ("Set EKF Origin Here") *(unsicher, ob QGroundControl diese Funktion bietet)*.

**Erledigt, wenn:** Das Skript meldet "Lokale Position vorhanden".

---

## 7. Testen in Stufen

**Wo:** Skript läuft auf dem **Laptop**. QGroundControl läuft parallel zur Überwachung. **Voraussetzung:** Schritte 1 bis 6 erledigt.

Das Skript `guided_test.py` kennt zwei Varianten:
- **Variante A** (`--pilot-takeoff`, **empfohlen**): Der Pilot armt und startet selbst in Loiter und schaltet in der Luft auf Guided. Das Skript fliegt 0,5 m vorwärts, zurück und landet.
- **Variante B** (ohne Option): Das Skript armt und startet selbst.

Ein Moduswechsel durch den Pilot beendet das Skript sofort. `Ctrl+C` am Laptop schickt "Landen".

| Stufe | Bedingung | Befehl | Erfolgreich, wenn |
|---|---|---|---|
| 1 | **Propeller ab**, Drohne am Tisch | `python guided_test.py --connect udpin:0.0.0.0:14551 --dry-run` | Verbindung, lokale Position, aktueller Modus werden angezeigt |
| 2 | **Propeller ab**, gearmt | Variante B ohne Propeller starten | Armen klappt; Skript meldet "Zielhöhe nicht erreicht" und landet (richtig, da ohne Propeller) |
| 3 | **Propeller ab** | Während Stufe 2 Schalter auf Loiter | Skript meldet "Pilot hat übernommen" und sendet nichts mehr |
| 4 | **Propeller ab** | Not-Aus während Stufe 2 | Motoren stoppen sofort |
| 5 | **Propeller ab** | WLAN am Laptop während Stufe 2 trennen | Failsafe greift (Land) nach einigen Sekunden |
| 6 | **Propeller dran**, gesicherter Raum | `python guided_test.py --connect udpin:0.0.0.0:14551 --pilot-takeoff` | Drohne fliegt 0,5 m vor, zurück und landet |
| 7 | wie 6 | Variante B | Drohne startet selbst auf 1 m, fliegt und landet |

**Nach jedem Flug:**
- Flight-Controller-Log herunterladen: **Analyze Tools → Log Download**. Die Logs sind die Grundlage für Aufgabe 6.
- Kurz notieren: Datum, Raum, Stufe, Ergebnis, Auffälligkeiten (Drift, Abbrüche, Eingriffe).

**Erledigt, wenn:** Stufe 6 mehrfach ohne Eingriff gelingt.

---

## 8. Ausblick: Hinderniserkennung mit MT-06

Aufgabe 2 nennt auch **Hinderniserkennung**. Dafür sind die MT-06-Einpunkt-LiDARs vorgesehen. Dieser Schritt hängt vom offenen Issue *"Wie lässt sich die 2D-Karte mit den vorhandenen LiDAR-Sensoren realisieren?"* ab, weil Anzahl, Ausrichtung und Anschluss der MT-06 noch geklärt werden.

Grundidee in ArduPilot *(alles unsicher, in der Doku zu [Proximity Sensors](https://ardupilot.org/copter/docs/common-proximity-landingpage.html) und [Object Avoidance](https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html) prüfen)*:

1. Jeden MT-06 als eigenen Abstandssensor anlegen (`RNGFND2_…`, `RNGFND3_…`) mit Ausrichtung `RNGFNDx_ORIENT` (vorne, links, rechts, hinten).
2. Die Abstandssensoren als Proximity-Quelle nutzen (`PRX1_TYPE` = RangeFinder).
3. Einfache Vermeidung einschalten (`AVOID_ENABLE`, `AVOID_MARGIN`). Sie bremst in Loiter vor Hindernissen.
4. Für Guided zusätzlich einen Ausweich-Algorithmus prüfen (`OA_TYPE`, z. B. BendyRuler).

---

## 9. Checkliste, Fehlersuche, offene Fragen

### Checkliste Aufgabe 2

- [ ] ArduPilot konfiguriert, Parameter als Datei gesichert und im Repo abgelegt
- [ ] MAVLink-Kette Flight Controller ↔ Pi ↔ Laptop funktioniert
- [ ] Loiter ohne GPS stabil (Schritt 1)
- [ ] Guided-Flug mit Skript erfolgreich (Schritt 7, Stufe 6)
- [ ] Manuelle Übernahme per Schalter getestet (Stufe 3)
- [ ] Not-Aus getestet (Stufe 4)
- [ ] Failsafe bei Funk- bzw. WLAN-Verlust getestet (Stufe 5)
- [ ] Hinderniserkennung mit MT-06 eingebunden (Schritt 8)
- [ ] Testprotokolle und Logs dokumentiert

### Fehlersuche

| Problem | Mögliche Ursache | Prüfen |
|---|---|---|
| MAVProxy auf dem Pi zeigt nichts | TX/RX vertauscht, falsche Baudrate, serielle Konsole noch aktiv | Kabel tauschen, `SERIALx_BAUD` und `--baudrate` vergleichen, `raspi-config` |
| QGroundControl findet die Drohne nicht über WLAN | falsche Laptop-IP, Firewall, Geräte in getrennten Netzen | IP neu prüfen, Firewall, eigener Router |
| Skript wartet ewig auf Heartbeat | falscher Port (14550 statt 14551) oder QGroundControl belegt ihn | `--connect udpin:0.0.0.0:14551` verwenden |
| "Moduswechsel auf GUIDED fehlgeschlagen" | kein EKF-Origin, keine Position | Schritt 6, Source-Set 3 aktiv? |
| Drohne driftet in Loiter oder Guided | Boden ohne Struktur, schlechtes Licht, Kompass gestört, Flusssensor nicht kalibriert | Schritt 1 wiederholen |
| Pi startet während des Betriebs neu | Stromversorgung reicht nicht | eigene Versorgung (Powerbank) testen |

### Offene Fragen

- Welcher UART am MicoAir743 V2 ist frei? (Pinbelegung prüfen)
- Reicht der 5-V-BEC für den Pi, oder braucht er eine eigene Versorgung?
- Welche Schalter der GX12 sind bereits belegt, und wie wird aktuell gearmt?
- Darf im Testgebäude ein eigener Router betrieben werden?
- Wie streng legt der Professor "autonom" für Aufgabe 2 aus: Ausführen von Befehlen (Guided) oder bereits selbstständige Zielwahl (eigentlich Aufgabe 4)?