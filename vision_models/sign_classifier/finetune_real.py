"""
Merges dataset_real/ (captured by capture_real_data.py) into a copy of the
synthetic dataset_yolo/ and fine-tunes YOLOv8n from scratch (COCO weights)
on the combination - closes the sim-to-real gap the pure-synthetic model
failed on.

Starts from yolov8n.pt again rather than continuing from the previous
synthetic-only best.pt: that checkpoint may have converged on spurious
shortcuts specific to the synthetic renderer's exact pixel statistics
(near-perfect synthetic val metrics but real-world failure is the
textbook symptom), so a clean COCO-pretrained start on the combined
dataset is the safer bet.

Real images are duplicated (REAL_OVERSAMPLE times) into the training set
so a few hundred real photos aren't drowned out by 4000 synthetic ones -
crude, but ultralytics has no simpler per-source sampling-ratio control.

Run with the IOAI2026 venv, after capture_real_data.py has produced a
reasonable amount of data (see that file's docstring for target counts):
    C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe finetune_real.py
"""
import random
import shutil
from pathlib import Path

from ultralytics import YOLO

SYNTHETIC_DIR = Path("dataset_yolo")
REAL_DIR = Path("dataset_real")
MERGED_DIR = Path("dataset_merged")
REAL_VAL_FRAC = 0.15
REAL_OVERSAMPLE = 5

IMGSZ = 320
EPOCHS = 60
BATCH = 32


def build_merged_dataset():
    if MERGED_DIR.exists():
        shutil.rmtree(MERGED_DIR)
    for split in ("train", "val"):
        (MERGED_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (MERGED_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)

    # copy synthetic as-is
    for split in ("train", "val"):
        for img in (SYNTHETIC_DIR / "images" / split).glob("*.jpg"):
            shutil.copy(img, MERGED_DIR / "images" / split / img.name)
            lbl = SYNTHETIC_DIR / "labels" / split / (img.stem + ".txt")
            shutil.copy(lbl, MERGED_DIR / "labels" / split / lbl.name)

    real_images = sorted((REAL_DIR / "images").glob("*.jpg"))
    if not real_images:
        raise FileNotFoundError(
            f"No images in {REAL_DIR/'images'} - run capture_real_data.py first and capture some real frames."
        )

    random.seed(0)
    random.shuffle(real_images)
    n_val = max(1, int(len(real_images) * REAL_VAL_FRAC))
    val_real = real_images[:n_val]
    train_real = real_images[n_val:]

    for img in val_real:
        shutil.copy(img, MERGED_DIR / "images" / "val" / img.name)
        lbl = REAL_DIR / "labels" / (img.stem + ".txt")
        shutil.copy(lbl, MERGED_DIR / "labels" / "val" / lbl.name)

    for img in train_real:
        lbl = REAL_DIR / "labels" / (img.stem + ".txt")
        for i in range(REAL_OVERSAMPLE):
            shutil.copy(img, MERGED_DIR / "images" / "train" / f"{img.stem}_dup{i}{img.suffix}")
            shutil.copy(lbl, MERGED_DIR / "labels" / "train" / f"{img.stem}_dup{i}.txt")

    n_train_total = len(list((MERGED_DIR / "images" / "train").glob("*.jpg")))
    n_val_total = len(list((MERGED_DIR / "images" / "val").glob("*.jpg")))
    print(f"merged dataset: {n_train_total} train ({len(train_real)} real x{REAL_OVERSAMPLE}), "
          f"{n_val_total} val ({len(val_real)} real)")


def write_data_yaml():
    names = "\n".join(f"  {i}: stage_{i+1}" for i in range(10))
    (MERGED_DIR / "data.yaml").write_text(f"""path: {MERGED_DIR.resolve().as_posix()}
train: images/train
val: images/val
names:
{names}
""")


def main():
    build_merged_dataset()
    write_data_yaml()

    model = YOLO("yolov8n.pt")
    model.train(
        data=str(MERGED_DIR / "data.yaml"),
        imgsz=IMGSZ,
        epochs=EPOCHS,
        batch=BATCH,
        project="runs",
        name="sign_yolo_real",
        patience=15,
        val=True,
    )
    metrics = model.val()
    print("final val metrics (on merged real+synthetic val set):", metrics.results_dict)

    model.export(format="onnx", imgsz=IMGSZ, opset=18)
    print("exported ONNX model under runs/sign_yolo_real/weights/")


if __name__ == "__main__":
    main()
