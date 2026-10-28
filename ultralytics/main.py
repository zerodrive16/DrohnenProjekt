import sys
import time
from datetime import datetime
import cv2
import torch
from ultralytics import YOLO


def open_camera(index: int = 0) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Could not open camera with index {index}")
        sys.exit(1)
    return cap


# Just check for available cameras in v4l2-ctl --list-devices
def find_camera(max_index: int = 4) -> int:
    for index in range(max_index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        ok, _ = cap.read() if cap.isOpened() else (False, None)
        cap.release()
        if ok:
            return index
    print("No available camera found.")
    sys.exit(1)


def main(index: int, model_name: str = "yolo11n.pt") -> None:
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Model: {model_name}, Device: {device}")
    model = YOLO(model_name)

    cap = open_camera(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    writer = None
    prev_time = time.time()
    fps = 0.0
    detect = True
    conf = 0.40

    print("-" * 30)
    print("Läuft. q/ESC = Stop, s = Screenshot, r = Record, d = Detection, +/- Confidence")
    print("-" * 30)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to capture frame")
            break

        display = frame
        n_det = 0
        if detect:
            results = model.predict(frame, conf=conf, imgsz=640, verbose=False)[0]
            display = results.plot()
            n_det = len(results.boxes)

        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - prev_time, 1e-6))
        prev_time = now

        status = (
            f"FPS: {fps:.1f} | Detection: {'on' if detect else 'off'} "
            f"| Confidence: conf>={conf:.2f} | Objects: {n_det}"
            + (" | Recording" if writer else "")
        )
        cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        if writer is not None:
            writer.write(display)

        cv2.imshow("Camera + Yolo", display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            break
        elif key == ord("d"):
            detect = not detect
        elif key in (ord("+"), ord("=")):
            conf = min(conf + 0.05, 0.95)
        elif key == ord("-"):
            conf = max(conf - 0.05, 0.05)
        elif key == ord("s"):
            name = datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"
            cv2.imwrite(name, display)
            print(f"Screenshot saved as {name}")
        elif key == ord("r"):
            if writer is None:
                name = datetime.now().strftime("%Y%m%d_%H%M%S") + ".avi"
                h, w = display.shape[:2]
                writer = cv2.VideoWriter(name, cv2.VideoWriter_fourcc(*"XVID"), 30, (w, h))
                print(f"Recording started: {name}")
            else:
                writer.release()
                writer = None
                print("Recording stopped")

    if writer is not None:
        writer.release()
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
    cam = int(sys.argv[1] if len(sys.argv) > 1 else find_camera())
    mdl = sys.argv[2] if len(sys.argv) > 2 else "yolo11n.pt"
    main(cam, mdl)
