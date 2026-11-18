"""Tests 1-3: what does the model see in the drone video?

Run:  python training/diagnose/diagnose_video.py output/flug.avi            # recorded video
      python training/diagnose/diagnose_video.py output/flug.avi path/best.pt
      python training/diagnose/diagnose_video.py 1                           # live grabber, index 1
      python training/diagnose/diagnose_video.py output/flug.avi yolov8m-worldv2.pt   # YOLO-World

Test 1 - image check: resolution, interlacing score, first frame saved as PNG.
Test 2 - threshold:   runs with a very low confidence and shows how confident the
                      model is on doors (seen but unsure, or not seen at all?).
Test 3 - deinterlace: every frame is run twice, original and deinterlaced, and
                      the results are compared.

Results: printed + example images in training/diagnose/output/<video name>/
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO

# ------------------------------------------------------------------
# SETTINGS
# ------------------------------------------------------------------
CONF = 0.05                   # very low on purpose: we want to see unsure detections too
LIVE_CONF = 0.40              # the threshold used in ultralytics/main.py
MAX_CAMERA_FRAMES = 600       # live mode stops after this many frames (~25 s)
EXAMPLE_EVERY = 50            # save an annotated example every N frames
IMAGE_SIZE = 640
WORLD_CLASSES = ["person", "door", "door handle", "chair", "table", "couch", "window"]  # YOLO-World only
# ------------------------------------------------------------------

HERE = Path(__file__).resolve().parent
DEFAULT_WEIGHTS = HERE.parent.parent / "models" / "merged_best.pt"
CAM_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2


def deinterlace(frame):
    """Keep only the even lines (one field) and scale back up: removes comb artifacts."""
    h, w = frame.shape[:2]
    return cv2.resize(frame[::2], (w, h), interpolation=cv2.INTER_LINEAR)


def comb_score(frame):
    """Neighbouring lines vs lines two apart. Progressive image: < 1, interlaced: > 1."""
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
    near = np.abs(g[1:] - g[:-1]).mean()
    far = np.abs(g[2:] - g[:-2]).mean()
    return near / max(far, 1e-6)


def open_source(source):
    if source.isdigit():
        cap = cv2.VideoCapture(int(source), CAM_BACKEND)
        name, limit = f"camera_{source}", MAX_CAMERA_FRAMES
    else:
        cap = cv2.VideoCapture(source)
        name, limit = Path(source).stem, None
    if not cap.isOpened():
        raise SystemExit(f"Could not open: {source}")
    return cap, name, limit


def new_stats():
    return {"door_conf": {}, "other": {}, "frames_door_any": 0, "frames_door_live": 0}


def collect(stats, result, names):
    best = 0.0
    for c, s in zip(result.boxes.cls.tolist(), result.boxes.conf.tolist()):
        name = names[int(c)]
        if "door" in name or "handle" in name:
            stats["door_conf"].setdefault(name, []).append(s)
            best = max(best, s)
        else:
            stats["other"][name] = stats["other"].get(name, 0) + 1
    stats["frames_door_any"] += best > 0
    stats["frames_door_live"] += best >= LIVE_CONF


def report(label, stats, frames):
    print(f"\n--- {label} ---")
    print(f"Frames with a door (conf >= {CONF}):      {stats['frames_door_any']}/{frames}")
    print(f"Frames with a door (conf >= {LIVE_CONF}, live): {stats['frames_door_live']}/{frames}")
    if not stats["door_conf"]:
        print("No door detections at all.")
    for name, confs in sorted(stats["door_conf"].items()):
        c = np.array(confs)
        bins = np.histogram(c, bins=[CONF, 0.15, 0.25, LIVE_CONF, 1.0])[0]
        print(f"  {name:<18} n={len(c):<5} median {np.median(c):.2f}  max {c.max():.2f}  "
              f"| {CONF}-0.15: {bins[0]}  0.15-0.25: {bins[1]}  0.25-{LIVE_CONF}: {bins[2]}  "
              f">={LIVE_CONF}: {bins[3]}")
    top = sorted(stats["other"].items(), key=lambda x: -x[1])[:8]
    print("Other classes (count):", ", ".join(f"{n} {k}" for n, k in top) or "-")


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    weights = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_WEIGHTS
    world = "world" in weights.name
    if not weights.exists() and not world:      # YOLO-World weights are downloaded automatically
        raise SystemExit(f"Model not found: {weights}")
    device = 0 if torch.cuda.is_available() else "cpu"
    model = YOLO(str(weights))
    if world:
        model.set_classes(WORLD_CLASSES)

    cap, name, limit = open_source(sys.argv[1])
    out = HERE / "output" / f"{name}_{weights.stem}"
    out.mkdir(parents=True, exist_ok=True)
    print(f"Model: {weights}, Device: {device}")
    print(f"Examples: {out}")

    stats = {"original": new_stats(), "deinterlaced": new_stats()}
    combs, frames = [], 0
    while limit is None or frames < limit:
        ok, frame = cap.read()
        if not ok:
            break
        if frames == 0:
            cv2.imwrite(str(out / "first_frame.png"), frame)
            h, w = frame.shape[:2]
        combs.append(comb_score(frame))

        for variant, img in (("original", frame), ("deinterlaced", deinterlace(frame))):
            r = model.predict(img, conf=CONF, imgsz=IMAGE_SIZE, device=device, verbose=False)[0]
            collect(stats[variant], r, model.names)
            if frames % EXAMPLE_EVERY == 0:
                cv2.imwrite(str(out / f"{frames:05d}_{variant}.jpg"), r.plot())
        frames += 1
        if frames % 100 == 0:
            print(f"  {frames} frames...")
    cap.release()

    if frames == 0:
        raise SystemExit("No frames read.")
    comb = float(np.mean(combs))

    print("\n" + "=" * 70)
    print("TEST 1 - image")
    print(f"  Resolution: {w}x{h}   (PAL analog: 720x576, NTSC: 720x480)")
    print(f"  Interlacing score: {comb:.2f}   (< 1.0 ok, > 1.0 = comb artifacts, compare Test 3)")
    print(f"  First frame: {out / 'first_frame.png'}  (look at it at 100% zoom)")
    print("\nTEST 2 - door confidence (original)")
    report("original", stats["original"], frames)
    print("\nTEST 3 - deinterlaced")
    report("deinterlaced", stats["deinterlaced"], frames)
    print("=" * 70)
    print("Reading the results:")
    print("  many doors at 0.15-0.40, few above  -> model sees doors but is unsure: lower conf + retrain")
    print("  hardly any doors even at 0.05        -> model does not see them: run val_degraded.py")
    print("  deinterlaced clearly better          -> deinterlace frames in main.py")
    print(f"  check the example images in {out} for doors labelled as other classes")


if __name__ == "__main__":
    main()
