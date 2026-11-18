"""Live detection on the grabber, a camera or a recorded video - runs on Windows and Linux.

Run:  python ultralytics/detect.py                                      # default grabber
      python ultralytics/detect.py 2                                    # camera index
      python ultralytics/detect.py /dev/video2                          # Linux device
      python ultralytics/detect.py training/diagnose/output/doors3.avi  # recorded video
      python ultralytics/detect.py 1 yolov8m-worldv2.pt                 # YOLO-World
q or ESC = quit.
"""
import sys
import cv2
import torch
from ultralytics import YOLO
from pathlib import Path 

MERGED_MODEL = Path(__file__).parent.parent / "models/merged_best.pt"
# Only used for YOLO-World models (e.g. yolov8m-worldv2.pt): searches only for these objects
WORLD_CLASSES = ["person", "door", "door handle", "chair", "table", "couch", "window"]

#CAM_BACKEND = cv2.CAP_V4L2
CAM_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2

# Default grabber per OS (override with the first argument)
DEFAULT_CAMERA = 1 if sys.platform == "win32" else "/dev/video0"


def open_camera(source) -> cv2.VideoCapture:
    if isinstance(source, str) and Path(source).is_file():
        cap = cv2.VideoCapture(source)              # recorded video, e.g. doors3.avi
    else:
        cap = cv2.VideoCapture(source, CAM_BACKEND)
    if not cap.isOpened():
        print(f"Could not open camera: {source}")
        sys.exit(1)
    return cap



def main(index, model_path: str = str(MERGED_MODEL)) -> None:
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Model: {model_path}, Device: {device}")
    model = YOLO(model_path)
    world = "world" in Path(model_path).name
    if world:
        model.set_classes(WORLD_CLASSES)

    cap = open_camera(index)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    conf = 0.25 if world else 0.40   # YOLO-World gives lower scores

    print("Läuft. q/ESC = Stop")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to capture frame")
            break

        results = model.predict(frame, conf=conf, imgsz=640, verbose=False)[0]
        display = results.plot()

        cv2.imshow("Camera + Yolo", display)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # look in system for v4l2-ctl --list-devices
    '''
    USB2.0 FHD UVC WebCam: USB2.0 F (usb-0000:63:00.4-1):
	/dev/video0
	/dev/video1
	/dev/video2
	/dev/video3

	/dev/media0
	/dev/media1
    '''

    # python ultralytics/main.py [kamera] [modell]
    #   kamera: Index (1), Gerät (/dev/video2) oder Videodatei (doors3.avi)
    arg = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CAMERA
    cam = int(arg) if isinstance(arg, str) and arg.isdigit() else arg
    mdl = sys.argv[2] if len(sys.argv) > 2 else str(MERGED_MODEL)
    main(cam, mdl)