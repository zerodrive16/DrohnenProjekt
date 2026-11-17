"""Step 2: build an 84-class starting model that still contains yolo26n's COCO knowledge.

Normally, training on 84 classes throws away the stock model's 80-class output
layers and starts them from random numbers. Here we instead copy every stock weight
into the bigger model (80 -> 84 classes), so at the start it behaves exactly like the
stock model for the 80 COCO classes, and the 4 door classes are new.

Run:  uv run python training/make_init_weights.py     (train.py does this for you)
"""
import copy
from pathlib import Path

import torch
from ultralytics import YOLO
from ultralytics.nn.tasks import DetectionModel

HERE = Path(__file__).resolve().parent
STOCK_MODEL = HERE.parent / "ultralytics" / "DoorDetect-Dataset" / "yolo26n.pt"
INIT_MODEL = HERE / "yolo26n_84cls_init.pt"
DOOR_NAMES = ["door", "handle", "cabinet door", "refrigerator door"]
NEW_NC = 84
OLD_NC = 80


def build_init_model():
    stock = YOLO(str(STOCK_MODEL))
    old_state = stock.model.state_dict()

    new = DetectionModel(copy.deepcopy(stock.model.yaml), nc=NEW_NC, verbose=False)
    new_state = new.state_dict()

    for key, tensor in new_state.items():
        old = old_state[key]
        if tensor.shape == old.shape:
            tensor.copy_(old)                       # same shape: copy everything
            continue
        # Class layers grew from 80 to 84 channels: copy the 80 old channels in...
        region = tuple(slice(0, s) for s in old.shape)
        tensor[region] = old
        # ...and make the 4 new input channels invisible to the 80 old outputs.
        if tensor.ndim == 4 and old.shape[0] == OLD_NC and old.shape[1] == OLD_NC:
            tensor[:OLD_NC, OLD_NC:] = 0
    new.load_state_dict(new_state)

    names = {i: stock.names[i] for i in range(OLD_NC)}
    names.update({OLD_NC + i: n for i, n in enumerate(DOOR_NAMES)})
    new.names = names

    stock.model = new
    stock.model.args = getattr(stock.model, "args", {})
    stock.save(str(INIT_MODEL))
    return stock


def verify():
    """Check the new model gives the same COCO detections as the stock model."""
    stock = YOLO(str(STOCK_MODEL))
    init = YOLO(str(INIT_MODEL))
    sample = sorted((HERE / "dataset" / "images" / "val").glob("coco_*.jpg"))[:20]
    if not sample:
        print("  (no dataset yet - skipping the check)")
        return
    device = 0 if torch.cuda.is_available() else "cpu"
    same = total = 0
    for img in sample:
        a = stock.predict(str(img), conf=0.4, device=device, verbose=False)[0]
        b = init.predict(str(img), conf=0.4, device=device, verbose=False)[0]
        ka = sorted((int(c), round(float(x))) for c, x in zip(a.boxes.cls, a.boxes.xyxy[:, 0])) if len(a.boxes) else []
        kb = sorted((int(c), round(float(x))) for c, x in zip(b.boxes.cls, b.boxes.xyxy[:, 0])) if len(b.boxes) else []
        total += max(len(ka), len(kb))
        same += len(set(ka) & set(kb))
    print(f"  check: {same}/{total} stock detections reproduced by the 84-class init model "
          f"(boxes within ~1px; fp16 rounding explains small differences)")


if __name__ == "__main__":
    build_init_model()
    print(f"Saved {INIT_MODEL}")
    verify()
