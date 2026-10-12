"""
Renders the synthetic sign-detection dataset (dataset_detector.py) to disk
in YOLO format, so it can be used to fine-tune a real YOLO model
(train_yolo.py) instead of the from-scratch single-object regression head
in detector_model.py - that model's bbox IoU (~0.45) was too weak, per
explicit feedback to use a proper detector architecture instead.

Layout (standard ultralytics format):
    dataset_yolo/
      images/train/*.jpg   images/val/*.jpg
      labels/train/*.txt   labels/val/*.txt   (YOLO format: "class cx cy w h",
                                                normalized 0-1; empty file = no object)
      data.yaml

Classes are 0-indexed stage numbers (class 0 = stage 1, ..., class 9 = stage 10) -
matches detector_model.py's convention.

Run with the IOAI2026 venv:
    C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe export_yolo_dataset.py
"""
import gc
from pathlib import Path

from PIL import Image

from dataset_detector import generate_detection_sample, NUM_STAGE_CLASSES

OUT_DIR = Path("dataset_yolo")
RENDER_SIZE = 320  # native render/output resolution written to disk - YOLO resizes to its own imgsz at train time
TRAIN_COUNT = 4000
VAL_COUNT = 600


def write_split(split, count, seed_base):
    img_dir = OUT_DIR / "images" / split
    lbl_dir = OUT_DIR / "labels" / split
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    for i in range(count):
        img, has_obj, cls, bbox = generate_detection_sample(
            rng=seed_base + i, render_size=RENDER_SIZE, output_size=RENDER_SIZE
        )
        Image.fromarray(img).save(img_dir / f"{i:06d}.jpg", quality=92)

        label_path = lbl_dir / f"{i:06d}.txt"
        if has_obj:
            cx, cy, w, h = bbox
            class_idx = cls - 1  # 1-10 -> 0-9
            label_path.write_text(f"{class_idx} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
        else:
            label_path.write_text("")  # empty label file = confirmed background, valid in YOLO format

        if (i + 1) % 500 == 0:
            print(f"{split}: {i+1}/{count}", flush=True)
            gc.collect()


def write_data_yaml():
    names = "\n".join(f"  {i}: stage_{i+1}" for i in range(NUM_STAGE_CLASSES))
    yaml_content = f"""path: {OUT_DIR.resolve().as_posix()}
train: images/train
val: images/val
names:
{names}
"""
    (OUT_DIR / "data.yaml").write_text(yaml_content)
    print(f"wrote {OUT_DIR / 'data.yaml'}")


def main():
    print("generating train split...")
    write_split("train", TRAIN_COUNT, seed_base=1_000_000)
    print("generating val split...")
    write_split("val", VAL_COUNT, seed_base=2_000_000)
    write_data_yaml()
    print("done")


if __name__ == "__main__":
    main()
