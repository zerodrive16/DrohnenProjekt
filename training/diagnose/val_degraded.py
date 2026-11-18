"""Test 4: is it the training or the analog image quality?

Evaluates the model on the held-out door images three times:
  clean  - the original DoorDetect val images
  light  - degraded like a good analog link
  strong - degraded like a bad analog link (noise, blur, interlacing, colour bleeding)

Run:  python training/diagnose/val_degraded.py                 # models/merged_best.pt
      python training/diagnose/val_degraded.py path/best.pt
Needs training/dataset/ (built by training/build_dataset.py).

Compare training/diagnose/output/degraded/<level>/images/ with a real grabber frame
(output/<video>/first_frame.png): "strong" should look at least as bad as the drone.
"""
import random
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO

# ------------------------------------------------------------------
# SETTINGS
# ------------------------------------------------------------------
# size = analog resolution the image is squeezed through, noise = gaussian sigma,
# blur = horizontal motion blur length (px), comb = line shift for interlacing (px),
# chroma = colour bleeding blur width (px), jpeg = compression quality
LEVELS = {
    "light":  dict(size=(720, 576), noise=4, blur=3, comb=0, chroma=5, jpeg=80),
    "strong": dict(size=(720, 576), noise=12, blur=9, comb=3, chroma=11, jpeg=50),
}
SEED = 0
# ------------------------------------------------------------------

HERE = Path(__file__).resolve().parent
DATASET = HERE.parent / "dataset"
DEFAULT_WEIGHTS = HERE.parent.parent / "models" / "merged_best.pt"
OUT = HERE / "output" / "degraded"
DOOR_OFFSET = 80


def degrade(img, p, rng):
    h, w = img.shape[:2]
    # squeeze through analog resolution and back
    img = cv2.resize(cv2.resize(img, p["size"], interpolation=cv2.INTER_AREA), (w, h))
    # colour bleeding: analog colour has much less resolution than brightness
    ycc = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    ycc[..., 1:] = cv2.blur(ycc[..., 1:], (p["chroma"], 1))
    img = cv2.cvtColor(ycc, cv2.COLOR_YCrCb2BGR)
    # horizontal motion blur (drone yaw)
    if p["blur"] > 1:
        img = cv2.filter2D(img, -1, np.ones((1, p["blur"]), np.float32) / p["blur"])
    # interlacing: odd lines come from a slightly later moment
    if p["comb"]:
        img[1::2] = np.roll(img[1::2], p["comb"], axis=1)
    # sensor / transmission noise
    noise = np.random.default_rng(rng.randrange(1 << 30)).normal(0, p["noise"], img.shape)
    img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    # grabber compression
    _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, p["jpeg"]])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def build_level(level, params, images, names_block):
    """Write degraded copies + labels; returns the yaml for this level."""
    rng = random.Random(SEED)
    root = OUT / level
    shutil.rmtree(root, ignore_errors=True)
    (root / "images").mkdir(parents=True)
    (root / "labels").mkdir()
    for src in images:
        img = cv2.imread(str(src))
        if img is None:
            continue
        cv2.imwrite(str(root / "images" / f"{src.stem}.jpg"), degrade(img, params, rng))
        label = DATASET / "labels" / "val" / f"{src.stem}.txt"
        shutil.copy(label, root / "labels" / f"{src.stem}.txt")
    yaml = root / "data.yaml"
    yaml.write_text(f"path: {root.as_posix()}\ntrain: images\nval: images\n{names_block}")
    return yaml


def door_rows(result, names):
    """(name, precision, recall, mAP50, mAP50-95) for each door class in the val set."""
    rows = {}
    for i, c in enumerate(result.ap_class_index):
        if c >= DOOR_OFFSET:
            rows[names[int(c)]] = result.box.class_result(i)
    return rows


def main():
    weights = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_WEIGHTS
    doors_yaml = DATASET / "data_doors.yaml"
    if not weights.exists():
        raise SystemExit(f"Model not found: {weights}")
    if not doors_yaml.exists():
        raise SystemExit("Dataset missing. Run first:  python training/build_dataset.py")

    images = [Path(p) for p in (DATASET / "val_doors.txt").read_text().splitlines() if p.strip()]
    names_block = doors_yaml.read_text().split("names:", 1)[1]
    names_block = "names:" + names_block

    print(f"Degrading {len(images)} door val images...")
    yamls = {"clean": doors_yaml}
    for level, params in LEVELS.items():
        yamls[level] = build_level(level, params, images, names_block)
    print(f"  examples: {OUT}")

    device = 0 if torch.cuda.is_available() else "cpu"
    model = YOLO(str(weights))
    kw = dict(device=device, verbose=False, plots=False, project=str(HERE / "output" / "runs"), exist_ok=True)
    results = {lvl: door_rows(model.val(data=str(y), name=f"val_{lvl}", **kw), model.names)
               for lvl, y in yamls.items()}

    print("\n" + "=" * 72)
    print(f"Model: {weights}")
    print(f"{'class':<18} {'level':<7} {'P':>6} {'R':>6} {'mAP50':>7} {'mAP50-95':>9}")
    for cls in sorted(results["clean"]):
        for lvl in yamls:
            if cls in results[lvl]:
                p, r, m50, m = results[lvl][cls]
                print(f"{cls:<18} {lvl:<7} {p:6.3f} {r:6.3f} {m50:7.3f} {m:9.3f}")
        print("-" * 72)
    print("Reading the results (look at 'door', mAP50):")
    print("  clean good (> 0.6), strong about the same -> image quality is not the problem;")
    print("        your doors / viewpoint differ from the dataset -> label own drone images")
    print("  clean good, strong clearly lower          -> domain gap: train with analog augmentation")
    print("  clean already bad                         -> training problem (COCO suppression / model size)")


if __name__ == "__main__":
    main()
