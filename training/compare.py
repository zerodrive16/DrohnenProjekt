"""Step 4: check that the merged model is still a good YOLO26 AND finds doors.

Run:  uv run python training/compare.py [path/to/best.pt]

Prints mAP50-95 for:
  - stock yolo26n on the COCO val images (the reference)
  - your merged model on the same COCO val images (should be close to the stock number)
  - your merged model on the held-out door images (door classes)
"""
import sys
from pathlib import Path

from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
STOCK = HERE.parent / "ultralytics" / "DoorDetect-Dataset" / "yolo26n.pt"
MERGED = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "runs" / "merged" / "weights" / "best.pt"
DATASET = HERE / "dataset"
DOOR_IDS = [80, 81, 82, 83]


def write_coco80_yaml(stock):
    """Stock model only knows 80 classes, so it needs a yaml with just those."""
    text = f"path: {DATASET.as_posix()}\ntrain: train.txt\nval: val_coco.txt\nnames:\n"
    text += "".join(f"  {i}: {stock.names[i]}\n" for i in range(80))
    path = DATASET / "data_coco80.yaml"
    path.write_text(text)
    return path


def main():
    if not MERGED.exists():
        raise SystemExit(f"Model not found: {MERGED}")

    stock = YOLO(str(STOCK))
    merged = YOLO(str(MERGED))
    kw = dict(device=0, verbose=False, plots=False, project=str(HERE / "runs"), exist_ok=True)

    r_stock = stock.val(data=str(write_coco80_yaml(stock)), name="cmp_stock", **kw)
    r_coco = merged.val(data=str(DATASET / "data_coco.yaml"), name="cmp_merged_coco", **kw)
    r_door = merged.val(data=str(DATASET / "data_doors.yaml"), name="cmp_merged_doors", **kw)

    # per-class mAP50-95 for the door classes (indexes into r_door.box.maps)
    print("\n" + "=" * 56)
    print(f"COCO val  stock  yolo26n : mAP50-95 {r_stock.box.map:.3f}   mAP50 {r_stock.box.map50:.3f}")
    print(f"COCO val  merged model   : mAP50-95 {r_coco.box.map:.3f}   mAP50 {r_coco.box.map50:.3f}")
    drop = r_stock.box.map - r_coco.box.map
    print(f"          difference     : {drop:+.3f}  (small = COCO knowledge preserved)")
    print("-" * 56)
    for i in DOOR_IDS:
        print(f"Door val  {merged.names[i]:<18}: mAP50-95 {r_door.box.maps[i]:.3f}")
    print("=" * 56)


if __name__ == "__main__":
    main()
