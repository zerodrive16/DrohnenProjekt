import sys
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO, YOLOWorld

# YOLO-World (Open Vocabulary): Klassen werden als Text vorgegeben (WORLD_CLASSES).
# Wird beim ersten Start automatisch heruntergeladen; benötigt das CLIP-Paket
# (ultralytics installiert es beim ersten Start selbst, sonst:
#  pip install git+https://github.com/ultralytics/CLIP.git).
# Varianten: yolov8s-worldv2.pt, yolov8m-worldv2.pt, yolov8l-worldv2.pt
#
# Alternativ Open Images V7 (601 feste Klassen, inkl. "Door" und "Door handle"):
#   yolov8n-oiv7.pt, yolov8s-oiv7.pt, yolov8m-oiv7.pt
DEFAULT_MODEL = "yolov8s-worldv2.pt"

# Nur für YOLO-World: frei wählbare Begriffe (Englisch funktioniert am besten).
WORLD_CLASSES = ["door", "door handle", "person", "chair", "table", "window"]

# Windows: DirectShow, Linux (z. B. Pi): V4L2
CAM_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2

# Standardkamera: wird über ihren Gerätenamen gesucht (Teilstring, Groß/Klein egal).
CAM_NAME = "USB Video"
# Falls der Name nicht gefunden wird (Windows ohne cv2-enumerate-cameras):
CAM_FALLBACK = 1 if sys.platform == "win32" else "/dev/video0"

# Nur für OIV7: diese Klassen anzeigen (None = alle 601 Klassen).
# Namen müssen exakt wie in Open Images geschrieben sein.
SHOW_CLASSES = None

CONF = 0.20


def find_camera(name: str):
    """Kamera über ihren Namen finden. -> Index (Windows) / Gerätepfad (Linux) oder None."""
    if sys.platform.startswith("linux"):
        # /sys/class/video4linux/videoN/name, nur Capture-Knoten (index 0) nehmen
        for dev in sorted(Path("/sys/class/video4linux").glob("video*")):
            try:
                dev_name = (dev / "name").read_text().strip()
                index = (dev / "index").read_text().strip()
            except OSError:
                continue
            if name.lower() in dev_name.lower() and index == "0":
                return f"/dev/{dev.name}"
        return None
    try:
        from cv2_enumerate_cameras import enumerate_cameras
    except ImportError:
        print("Hinweis: für die Suche nach Kameranamen unter Windows "
              "'pip install cv2-enumerate-cameras' ausführen.")
        return None
    for cam in enumerate_cameras(CAM_BACKEND):
        if name.lower() in cam.name.lower():
            return cam.index
    return None


def open_camera(source) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(source, CAM_BACKEND)
    if not cap.isOpened():
        print(f"Could not open camera: {source}")
        sys.exit(1)
    return cap


def class_ids(model, wanted):
    """Klassennamen -> IDs; unbekannte Namen werden gemeldet."""
    if not wanted:
        return None
    lookup = {name.lower(): i for i, name in model.names.items()}
    ids = []
    for name in wanted:
        if name.lower() in lookup:
            ids.append(lookup[name.lower()])
        else:
            print(f"Klasse nicht im Modell: {name!r}")
    return ids or None


def main(source, model_path: str = DEFAULT_MODEL) -> None:
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Model: {model_path}, Device: {device}")
    if "world" in model_path.lower():
        model = YOLOWorld(model_path)
        model.set_classes(WORLD_CLASSES)
        classes = None  # Filter steckt bereits in set_classes
        print(f"YOLO-World Klassen: {WORLD_CLASSES}")
    else:
        model = YOLO(model_path)
        classes = class_ids(model, SHOW_CLASSES)

    cap = open_camera(source)

    print("Läuft. q/ESC = Stop")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to capture frame")
            break

        results = model.predict(frame, conf=CONF, imgsz=640, classes=classes,
                                device=device, verbose=False)[0]
        display = results.plot()

        cv2.imshow("Camera + Yolo", display)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Aufruf:  python main.py [kamera] [modell]
    #   ohne kamera: Kamera mit dem Namen CAM_NAME ("USB Video") suchen
    #   Windows: kamera = 0, 1, 2 ...   (0 meist eingebaute Webcam)
    #   Linux:   kamera = /dev/video0   (siehe: v4l2-ctl --list-devices)
    #   kamera = Name, z. B. "Integrated Camera"
    cam = sys.argv[1] if len(sys.argv) > 1 else CAM_NAME
    if isinstance(cam, str) and cam.isdigit():
        cam = int(cam)
    elif isinstance(cam, str) and not cam.startswith("/dev/"):
        found = find_camera(cam)
        if found is None:
            print(f"Kamera {cam!r} nicht gefunden - nehme {CAM_FALLBACK}")
            found = CAM_FALLBACK
        else:
            print(f"Kamera {cam!r}: {found}")
        cam = found

    mdl = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_MODEL
    main(cam, mdl)
