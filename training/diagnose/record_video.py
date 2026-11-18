"""Record a test video from the grabber, so all diagnose tests run on the same footage.

Run:  python training/diagnose/record_video.py              # camera index 1
      python training/diagnose/record_video.py 0            # other camera index
      python training/diagnose/record_video.py 1 flur.avi   # own file name
q or ESC = stop recording.

Saved as MJPG .avi in high quality, so the video looks like the live image.
"""
import sys
import time
from pathlib import Path

import cv2

# ------------------------------------------------------------------
# SETTINGS
# ------------------------------------------------------------------
CAMERA_INDEX = 1              # grabber index (see ultralytics/find_camera.py)
JPEG_QUALITY = 95             # MJPG quality, high = close to the live image
# ------------------------------------------------------------------

HERE = Path(__file__).resolve().parent
OUT = HERE / "output"
CAM_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2


def main():
    index = int(sys.argv[1]) if len(sys.argv) > 1 else CAMERA_INDEX
    OUT.mkdir(exist_ok=True)
    path = Path(sys.argv[2]) if len(sys.argv) > 2 else OUT / f"flug_{time.strftime('%Y%m%d_%H%M%S')}.avi"

    cap = cv2.VideoCapture(index, CAM_BACKEND)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera with index {index}")
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Failed to capture frame")

    h, w = frame.shape[:2]
    fps = cap.get(cv2.CAP_PROP_FPS)
    fps = fps if 1 < fps < 120 else 25.0
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h))
    writer.set(cv2.VIDEOWRITER_PROP_QUALITY, JPEG_QUALITY)
    print(f"Recording {w}x{h} @ {fps:.0f} fps -> {path}")
    print("q/ESC = Stop")

    frames = 0
    while ok:
        writer.write(frame)
        frames += 1
        cv2.imshow("Recording", frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
        ok, frame = cap.read()

    cap.release()
    writer.release()
    cv2.destroyAllWindows()
    print(f"Saved {frames} frames ({frames / fps:.0f} s): {path}")
    print(f"Next: python training/diagnose/diagnose_video.py {path}")


if __name__ == "__main__":
    main()
