"""Step 1: build the merged dataset (COCO 80 classes + 4 door classes = 84).

What it does (no manual labeling needed):
  1. Downloads the official COCO labels and a random subset of COCO images.
  2. Runs the stock yolo26n.pt over your door images to label the COCO objects
     (person, chair, ...) that your door labels are missing.
  3. Adds your real door labels on top, with ids shifted to 80-83.
  4. Writes data.yaml + train/val lists into training/dataset/.

Run:  uv run python training/build_dataset.py
Re-running is safe: downloaded files are reused, the dataset folder is rebuilt.
"""
import random
import shutil
import subprocess
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO

# ------------------------------------------------------------------
# SETTINGS
# ------------------------------------------------------------------
COCO_TRAIN_IMAGES = 5000      # random COCO train images used as "replay" data
COCO_VAL_IMAGES = 500         # random COCO val images (to check COCO still works)
DOOR_REPEAT = 2               # door images are listed this many times per epoch
VAL_PERCENT = 20              # % of door images held out for testing
PSEUDO_CONF = 0.50            # keep stock-model detections above this confidence
PREVIEW_COUNT = 30            # annotated pseudo-label examples saved for you to skim
SEED = 0
# ------------------------------------------------------------------

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DOOR_DIR = ROOT / "ultralytics" / "DoorDetect-Dataset"
STOCK_MODEL = DOOR_DIR / "yolo26n.pt"
DOOR_NAMES = ["door", "handle", "cabinet door", "refrigerator door"]
COCO_OFFSET = 80              # door class 0 becomes 80, etc.

DOWNLOADS = HERE / "downloads"
OUT = HERE / "dataset"
LABELS_URL = "https://github.com/ultralytics/assets/releases/download/v0.0.0/coco2017labels.zip"
IMAGE_URL = "http://images.cocodataset.org/{split}/{name}.jpg"


def download(url, dest):
    if dest.exists() and dest.stat().st_size > 0:
        return True
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    # curl is used instead of urllib: Python here fails the SSL certificate check
    for _ in range(3):
        done = subprocess.run(["curl", "-sSfL", "--retry", "2", "-o", str(tmp), url],
                              capture_output=True)
        if done.returncode == 0 and tmp.exists() and tmp.stat().st_size > 0:
            tmp.replace(dest)
            return True
    return False


def get_coco_labels():
    """Download + extract the official COCO labels (YOLO txt format)."""
    target = DOWNLOADS / "coco" / "labels"
    if not (target / "train2017").exists():
        zip_path = DOWNLOADS / "coco2017labels.zip"
        print("Downloading COCO labels (~50 MB)...")
        if not download(LABELS_URL, zip_path):
            raise SystemExit("Could not download the COCO labels.")
        with zipfile.ZipFile(zip_path) as z:
            z.extractall(DOWNLOADS)
    return target


def pick_coco_images(labels_dir, split, count, rng):
    """Choose random COCO images that have at least one object."""
    names = sorted(p.stem for p in (labels_dir / split).glob("*.txt") if p.stat().st_size > 0)
    rng.shuffle(names)
    return names[:count]


def fetch_coco_split(labels_dir, split, names, group):
    """Download the images for `names` and copy images + labels into OUT/<group>."""
    img_dir = DOWNLOADS / "coco_images" / split
    jobs = [(IMAGE_URL.format(split=split, name=n), img_dir / f"{n}.jpg") for n in names]
    print(f"Downloading {len(jobs)} COCO {split} images (skips ones already downloaded)...")
    with ThreadPoolExecutor(16) as pool:
        ok = list(pool.map(lambda j: download(*j), jobs))
    paths = []
    for n, good in zip(names, ok):
        if not good:
            continue
        new = f"coco_{n}"
        shutil.copy(img_dir / f"{n}.jpg", OUT / "images" / group / f"{new}.jpg")
        shutil.copy(labels_dir / split / f"{n}.txt", OUT / "labels" / group / f"{new}.txt")
        paths.append(OUT / "images" / group / f"{new}.jpg")
    print(f"  {len(paths)}/{len(names)} ok")
    return paths


def pseudo_label_doors(door_pics, group, stock, device):
    """Copy door images; label = stock-model COCO boxes + real door boxes (+80)."""
    paths, kept = [], 0
    for i in range(0, len(door_pics), 32):
        batch = door_pics[i:i + 32]
        results = stock.predict([str(p) for p in batch], conf=PSEUDO_CONF, imgsz=640,
                                device=device, verbose=False)
        for pic, res in zip(batch, results):
            lines = []
            for box in res.boxes:                      # stock COCO detections
                x, y, w, h = box.xywhn[0].tolist()
                lines.append(f"{int(box.cls)} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")
            kept += len(lines)
            real = (DOOR_DIR / "labels" / f"{pic.stem}.txt").read_text().splitlines()
            for line in real:                          # your real door labels
                parts = line.split()
                if len(parts) == 5:
                    lines.append(" ".join([str(int(parts[0]) + COCO_OFFSET)] + parts[1:]))

            new = f"door_{pic.stem}"
            dest = OUT / "images" / group / f"{new}{pic.suffix.lower()}"
            shutil.copy(pic, dest)
            (OUT / "labels" / group / f"{new}.txt").write_text("\n".join(lines))
            paths.append(dest)
    print(f"  {group}: {len(paths)} door images, {kept} pseudo-labeled COCO objects added")
    return paths


def save_previews(paths, names):
    """Draw labels on a few door images so you can check the pseudo-labels."""
    prev = HERE / "pseudo_label_previews"
    shutil.rmtree(prev, ignore_errors=True)
    prev.mkdir()
    for p in paths[:PREVIEW_COUNT]:
        img = cv2.imread(str(p))
        if img is None:
            continue
        h, w = img.shape[:2]
        for line in (OUT / "labels" / p.parent.name / f"{p.stem}.txt").read_text().splitlines():
            c, x, y, bw, bh = line.split()
            c, x, y, bw, bh = int(c), float(x) * w, float(y) * h, float(bw) * w, float(bh) * h
            color = (0, 0, 255) if c >= COCO_OFFSET else (0, 200, 0)   # red = door, green = COCO
            cv2.rectangle(img, (int(x - bw / 2), int(y - bh / 2)),
                          (int(x + bw / 2), int(y + bh / 2)), color, 2)
            cv2.putText(img, names[c], (int(x - bw / 2), max(int(y - bh / 2) - 4, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.imwrite(str(prev / p.name), img)
    print(f"  previews: {prev}  (green = COCO pseudo-label, red = your door labels)")


def write_list(path, files):
    path.write_text("\n".join(p.as_posix() for p in files))


def main():
    rng = random.Random(SEED)
    device = 0 if torch.cuda.is_available() else "cpu"
    if device == "cpu":
        print("WARNING: no GPU found, pseudo-labeling will be slow (training will refuse to run).")

    if not STOCK_MODEL.exists():
        raise SystemExit(f"Stock model not found: {STOCK_MODEL}")
    stock = YOLO(str(STOCK_MODEL))
    coco_names = [stock.names[i] for i in range(80)]
    all_names = coco_names + DOOR_NAMES

    shutil.rmtree(OUT, ignore_errors=True)
    for kind in ("images", "labels"):
        for group in ("train", "val"):
            (OUT / kind / group).mkdir(parents=True)

    # 1) COCO replay data
    labels_dir = get_coco_labels()
    coco_train = fetch_coco_split(labels_dir, "train2017",
                                  pick_coco_images(labels_dir, "train2017", COCO_TRAIN_IMAGES, rng), "train")
    coco_val = fetch_coco_split(labels_dir, "val2017",
                                pick_coco_images(labels_dir, "val2017", COCO_VAL_IMAGES, rng), "val")

    # 2) Door images: split, pseudo-label, add real door labels
    door_pics = sorted(p for p in (DOOR_DIR / "images").iterdir()
                       if p.suffix.lower() in {".jpg", ".jpeg", ".png"}
                       and (DOOR_DIR / "labels" / f"{p.stem}.txt").exists())
    rng.shuffle(door_pics)
    n_val = len(door_pics) * VAL_PERCENT // 100
    print("Pseudo-labeling door images with the stock model...")
    door_val = pseudo_label_doors(door_pics[:n_val], "val", stock, device)
    door_train = pseudo_label_doors(door_pics[n_val:], "train", stock, device)
    save_previews(door_train, all_names)

    # 3) Lists + yaml files (door train images are repeated DOOR_REPEAT times)
    write_list(OUT / "train.txt", coco_train + door_train * DOOR_REPEAT)
    write_list(OUT / "val.txt", coco_val + door_val)
    write_list(OUT / "val_coco.txt", coco_val)
    write_list(OUT / "val_doors.txt", door_val)

    for name, val in (("data", "val.txt"), ("data_coco", "val_coco.txt"), ("data_doors", "val_doors.txt")):
        text = f"path: {OUT.as_posix()}\ntrain: train.txt\nval: {val}\nnames:\n"
        text += "".join(f"  {i}: {n}\n" for i, n in enumerate(all_names))
        (OUT / f"{name}.yaml").write_text(text)

    print("\nDone.")
    print(f"  train: {len(coco_train)} COCO + {len(door_train)} door (x{DOOR_REPEAT}) images")
    print(f"  val:   {len(coco_val)} COCO + {len(door_val)} door images")
    print(f"  data:  {OUT / 'data.yaml'}")
    print("Next: skim training/pseudo_label_previews/, then run training/train.py")


if __name__ == "__main__":
    main()
