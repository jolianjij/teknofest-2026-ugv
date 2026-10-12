"""
Trains DrivableSegNet (see cnn_model.py) on (image, mask) pairs from a
directory. NOT RUNNABLE YET in any meaningful sense - there is no dataset
directory yet. This is infrastructure, staged for the moment real
front-camera frames exist, via one of:

  1. Real frames captured from the actual parkour/course terrain, with
     masks from ground_plane_heuristic.py's pseudo-labeler (see that
     file's header - it needs zero manual annotation).
  2. The same frames with hand-corrected masks, if pseudo-labels prove too
     noisy once actually inspected against real terrain.

Expected directory layout:
    dataset/
      images/*.png   (or .jpg)
      masks/*.png    (same basename as the matching image; 0=not drivable,
                      255=drivable, single channel)

Run with the IOAI2026 venv:
    C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe train.py --data dataset/
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from PIL import Image

sys.path.insert(0, ".")
from cnn_model import DrivableSegNet, INPUT_SIZE

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)  # ImageNet stats, matches the pretrained backbone
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class SegDataset(Dataset):
    def __init__(self, root, augment=True):
        self.root = Path(root)
        self.images = sorted((self.root / "images").glob("*.png")) + sorted((self.root / "images").glob("*.jpg"))
        if not self.images:
            raise FileNotFoundError(
                f"No images found under {self.root/'images'} - this pipeline needs real captured "
                f"frames before it can train anything, see this file's module docstring."
            )
        self.augment = augment

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img_path = self.images[idx]
        # Masks are written as PNG (lossless - a JPEG-compressed label would
        # introduce grey fringes at every boundary), so match on stem rather
        # than on the image's own extension.
        mask_path = next(
            (p for p in (
                self.root / "masks" / f"{img_path.stem}.png",
                self.root / "masks" / f"{img_path.stem}.jpg",
                self.root / "masks" / img_path.name,
            ) if p.exists()),
            None,
        )
        if mask_path is None:
            raise FileNotFoundError(f"no mask found for {img_path.name}")
        img = Image.open(img_path).convert("RGB").resize((INPUT_SIZE, INPUT_SIZE), Image.BICUBIC)
        mask = Image.open(mask_path).convert("L").resize((INPUT_SIZE, INPUT_SIZE), Image.NEAREST)

        img_arr = np.asarray(img, dtype=np.float32) / 255.0
        img_arr = (img_arr - MEAN) / STD
        img_arr = np.transpose(img_arr, (2, 0, 1))

        mask_arr = (np.asarray(mask) > 127).astype(np.int64)

        return torch.from_numpy(img_arr.astype(np.float32)), torch.from_numpy(mask_arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="dataset root containing images/ and masks/")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch-size", type=int, default=8)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--val-frac", type=float, default=0.15)
    args = ap.parse_args()

    print(f"device: {DEVICE}")
    ds = SegDataset(args.data)
    n_val = max(1, int(len(ds) * args.val_frac))
    n_train = len(ds) - n_val
    train_ds, val_ds = torch.utils.data.random_split(ds, [n_train, n_val])
    print(f"dataset: {len(ds)} frames ({n_train} train / {n_val} val)")

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, num_workers=0)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False, num_workers=0)

    model = DrivableSegNet(pretrained=True).to(DEVICE)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    criterion = nn.CrossEntropyLoss()

    best_iou = 0.0
    for epoch in range(1, args.epochs + 1):
        model.train()
        running_loss = 0.0
        for x, y in train_loader:
            x, y = x.to(DEVICE), y.to(DEVICE)
            opt.zero_grad()
            loss = criterion(model(x), y)
            loss.backward()
            opt.step()
            running_loss += loss.item() * x.size(0)
        sched.step()

        iou = evaluate_iou(model, val_loader)
        print(f"epoch {epoch:2d}/{args.epochs}  loss={running_loss/len(train_ds):.4f}  val_drivable_iou={iou:.4f}")
        if iou > best_iou:
            best_iou = iou
            torch.save(model.state_dict(), "drivable_segnet_best.pt")

    print(f"best val IoU (drivable class): {best_iou:.4f}")
    export_onnx()


def evaluate_iou(model, loader):
    model.eval()
    inter, union = 0, 0
    with torch.no_grad():
        for x, y in loader:
            x, y = x.to(DEVICE), y.to(DEVICE)
            pred = model(x).argmax(1)
            inter += ((pred == 1) & (y == 1)).sum().item()
            union += ((pred == 1) | (y == 1)).sum().item()
    model.train()
    return inter / union if union else 0.0


def export_onnx():
    model = DrivableSegNet(pretrained=False)
    model.load_state_dict(torch.load("drivable_segnet_best.pt", map_location="cpu"))
    model.eval()
    dummy = torch.randn(1, 3, INPUT_SIZE, INPUT_SIZE)
    torch.onnx.export(
        model, dummy, "drivable_segnet.onnx",
        input_names=["input"], output_names=["logits"],
        dynamic_axes={"input": {0: "batch"}, "logits": {0: "batch"}},
        opset_version=18,  # see sign_classifier/train.py's export_onnx comment - torch 2.10 targets 18 natively
    )
    print("exported drivable_segnet.onnx")


if __name__ == "__main__":
    main()
