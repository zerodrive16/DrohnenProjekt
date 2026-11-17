"""Run the finetuned merged model (COCO + doors) on the camera.

Run:  uv run python training/run_merged.py                 # first camera found
      uv run python training/run_merged.py 1                # camera index 1
      uv run python training/run_merged.py 0 path/best.pt   # other weights
q or ESC = quit.
"""
import sys
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
DEFAULT_WEIGHTS = HERE.parent / "models" / "merged_best.pt"
CAM_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
CONF = 0.40


def find_camera(max_index=4):
    for index in range(max_index):
        cap = cv2.VideoCapture(index, CAM_BACKEND)
        ok, _ = cap.read() if cap.isOpened() else (False, None)
        cap.release()
        if ok:
            return index
    raise SystemExit("No available camera found.")


def main():
    index = int(sys.argv[1]) if len(sys.argv) > 1 else find_camera()
    weights = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_WEIGHTS
    if not weights.exists():
        raise SystemExit(f"Model not found: {weights}\nTrain first with training/train.py")

    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Model: {weights}, Device: {device}")
    model = YOLO(str(weights))

    cap = cv2.VideoCapture(index, CAM_BACKEND)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera with index {index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("Läuft. q/ESC = Stop")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to capture frame")
            break
        results = model.predict(frame, conf=CONF, imgsz=640, device=device, verbose=False)[0]
        cv2.imshow("Merged model (COCO + doors)", results.plot())
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
