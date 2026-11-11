import sys
import cv2
import torch
from ultralytics import YOLO
from pathlib import Path 

DOOR_MODEL = Path(__file__).parent / "DoorDetect-Dataset/runs/doors/weights/best.pt"

# Camera backend: use ONE of the two lines, comment out the other
#CAM_BACKEND = cv2.CAP_DSHOW   # Windows: DirectShow
CAM_BACKEND = cv2.CAP_V4L2  # Linux: V4L2


def open_camera(index: int = 0) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, CAM_BACKEND)
    if not cap.isOpened():
        print(f"Could not open camera with index {index}")
        sys.exit(1)
    return cap


# Probes camera indices (Linux: see v4l2-ctl --list-devices)
def find_camera(max_index: int = 4) -> int:
    for index in range(max_index):
        cap = cv2.VideoCapture(index, CAM_BACKEND)
        ok, _ = cap.read() if cap.isOpened() else (False, None)
        cap.release()
        if ok:
            return index
    print("No available camera found.")
    sys.exit(1)


def main(index: int, model_name: str = "yolo26n.pt") -> None:
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Model: {model_name}, Device: {device}")
    model = YOLO(model_name)
    door_model = YOLO(str(DOOR_MODEL))

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
        door_results = door_model.predict(frame, conf=conf, imgsz=640, verbose=False)[0]
        display = door_results.plot(img=results.plot())

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
    cam = int(sys.argv[1]) if len(sys.argv) > 1 else 4  # /dev/video4
    mdl = sys.argv[2] if len(sys.argv) > 2 else "yolo26m-objv1-150.pt"
    main(cam, mdl)
