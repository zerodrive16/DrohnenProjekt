import sys
import cv2
import torch
from ultralytics import YOLO
from pathlib import Path 

MERGED_MODEL = Path(__file__).parent.parent / "models/merged_best.pt"

CAM_BACKEND = cv2.CAP_V4L2


def open_camera(source) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(source, CAM_BACKEND)
    if not cap.isOpened():
        print(f"Could not open camera: {source}")
        sys.exit(1)
    return cap



def main(index, model_path: str = str(MERGED_MODEL)) -> None:
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Model: {model_path}, Device: {device}")
    model = YOLO(model_path)

    cap = open_camera(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    conf = 0.40

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

    cam = "/dev/video0"
    mdl = sys.argv[2] if len(sys.argv) > 2 else str(MERGED_MODEL)
    main(cam, mdl)