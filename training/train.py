"""Step 3: train ONE model that detects COCO objects + doors (84 classes) on the GPU.

Run:  uv run python training/train.py
Quick test (about 1 minute, checks everything works):  uv run python training/train.py smoke

Result: training/runs/merged/weights/best.pt
"""
import sys
from pathlib import Path

import torch
from ultralytics import YOLO

from make_init_weights import INIT_MODEL, build_init_model

# ------------------------------------------------------------------
# SETTINGS
# ------------------------------------------------------------------
EPOCHS = 50                 # ~1 hour on an RTX 5070
IMAGE_SIZE = 640
BATCH_SIZE = 16             # lower to 8 if you get "out of memory"
LEARNING_RATE = 0.0005      # small on purpose: gentle fine-tuning keeps COCO knowledge
# ------------------------------------------------------------------

HERE = Path(__file__).resolve().parent
DATA = HERE / "dataset" / "data.yaml"


def main():
    if not torch.cuda.is_available():
        raise SystemExit("No GPU found - refusing to train on the CPU (it would take days).\n"
                         "Your PyTorch is the CPU version or the NVIDIA driver is missing.")
    print(f"GPU: {torch.cuda.get_device_name(0)}")

    if not DATA.exists():
        raise SystemExit("Dataset missing. Run first:  uv run python training/build_dataset.py")
    if not INIT_MODEL.exists():
        build_init_model()

    smoke = len(sys.argv) > 1 and sys.argv[1] == "smoke"
    model = YOLO(str(INIT_MODEL))
    model.train(
        data=str(DATA),
        epochs=1 if smoke else EPOCHS,
        fraction=0.05 if smoke else 1.0,
        imgsz=IMAGE_SIZE,
        batch=BATCH_SIZE,
        device=0,
        optimizer="AdamW",
        lr0=LEARNING_RATE,
        lrf=0.1,
        warmup_epochs=1,
        patience=15,
        workers=4,
        flipud=0.0,                  # doors are upright
        project=str(HERE / "runs"),
        name="smoke" if smoke else "merged",
        exist_ok=True,
    )
    print(f"\nDone. Your model: {HERE / 'runs' / ('smoke' if smoke else 'merged') / 'weights' / 'best.pt'}")
    print("Next: uv run python training/compare.py")


if __name__ == "__main__":
    main()
