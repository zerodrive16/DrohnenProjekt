# Testvideos (Grabber, 640x480, 25 fps)

| Datei | Inhalt |
|---|---|
| `doors3.mp4` | Flur mit Türen, Drohne in der Hand, **mit** ArduPilot-OSD (`OSD_TYPE=1`), 100 s |
| `doors5.mp4` | gleiche Art Strecke, **ohne** OSD (`OSD_TYPE=0`), 91 s |

Aufgenommen mit `record_video.py`, für das Repo nach H.264 (crf 20) komprimiert.

Abspielen mit Erkennung:

    python ultralytics/detect.py training/diagnose/videos/doors5.mp4

Diagnose:

    python training/diagnose/diagnose_video.py training/diagnose/videos/doors5.mp4

## Ergebnis doors3 (mit OSD), Modell `models/merged_best.pt`

| | Original | Deinterlaced |
|---|---|---|
| Frames mit Tür, conf >= 0.05 | 1907 / 2508 | 2049 / 2508 |
| Frames mit Tür, conf >= 0.40 (live) | 548 / 2508 | 958 / 2508 |
| Median-Konfidenz `door` | 0.18 | 0.17 |

- Das Modell sieht die Türen meist, ist aber unsicher (die meisten Detektionen bei 0.05-0.40).
- `handle` nie über 0.30.
- Viele Fehlerkennungen im Flur: `truck` 1215, `car` 926 (z. B. Glastür mit Reflexionen).
- Das OSD (Text, Fadenkreuz) liegt quer über den Türen, dazu kommt eine starke Fischaugen-Verzerrung.
- Interlacing-Score 0.59, also kein Kammeffekt. Deinterlacen hilft trotzdem (wirkt wie ein Weichzeichner).
