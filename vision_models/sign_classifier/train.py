"""
Trains SignCNN on procedurally-generated synthetic sign images (see
dataset.py) and exports both a PyTorch checkpoint and an ONNX model for
deployment via onnxruntime in sign_detector.py (CPU inference, see
CLAUDE.md - sign detection stays off the NPU).

Run with the IOAI2026 venv (has torch+cuda, onnx, onnxruntime already):
    C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe train.py

Since data is generated on-the-fly (effectively infinite, never repeats),
there is no fixed training set to overfit to - the "epoch" size below is
just a batching convenience. A separate fixed-seed validation set IS held
constant across epochs so accuracy is comparably tracked run-to-run.
"""
import sys
import time

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

sys.path.insert(0, ".")
from dataset import generate_sample, IMG_SIZE
from model import SignCNN, NUM_CLASSES

SAMPLES_PER_EPOCH = 8000
VAL_SAMPLES = 1000
BATCH_SIZE = 128
EPOCHS = 18
LR = 1e-3
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

MEAN = np.array([0.5, 0.5, 0.5], dtype=np.float32)
STD = np.array([0.25, 0.25, 0.25], dtype=np.float32)


class SyntheticSignDataset(Dataset):
    """Infinite synthetic dataset - __getitem__ ignores idx and always
    generates a fresh random sample, except when seed_base is set (used
    for the fixed validation set)."""

    def __init__(self, length, seed_base=None):
        self.length = length
        self.seed_base = seed_base

    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        rng = (self.seed_base + idx) if self.seed_base is not None else None
        img, label = generate_sample(rng)
        img = (img.astype(np.float32) / 255.0 - MEAN) / STD
        img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        return torch.from_numpy(img), label


def evaluate(model, loader):
    model.eval()
    correct, total = 0, 0
    per_class_correct = np.zeros(NUM_CLASSES)
    per_class_total = np.zeros(NUM_CLASSES)
    with torch.no_grad():
        for x, y in loader:
            x, y = x.to(DEVICE), y.to(DEVICE)
            pred = model(x).argmax(1)
            correct += (pred == y).sum().item()
            total += y.size(0)
            for c in range(NUM_CLASSES):
                mask = y == c
                per_class_total[c] += mask.sum().item()
                per_class_correct[c] += (pred[mask] == c).sum().item()
    model.train()
    return correct / total, per_class_correct, per_class_total


def main():
    print(f"device: {DEVICE}")
    train_ds = SyntheticSignDataset(SAMPLES_PER_EPOCH)
    val_ds = SyntheticSignDataset(VAL_SAMPLES, seed_base=999_000)  # fixed, disjoint seed range from training

    # num_workers=0: Windows spawn-based multiprocessing crashed the workers
    # here, and on-the-fly generation is cheap enough single-process anyway.
    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True, num_workers=0)
    val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False, num_workers=0)

    model = SignCNN().to(DEVICE)
    opt = torch.optim.Adam(model.parameters(), lr=LR)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=EPOCHS)
    criterion = nn.CrossEntropyLoss()

    best_acc = 0.0
    for epoch in range(1, EPOCHS + 1):
        t0 = time.time()
        running_loss = 0.0
        for x, y in train_loader:
            x, y = x.to(DEVICE), y.to(DEVICE)
            opt.zero_grad()
            loss = criterion(model(x), y)
            loss.backward()
            opt.step()
            running_loss += loss.item() * x.size(0)
        sched.step()

        acc, per_c, per_t = evaluate(model, val_loader)
        print(f"epoch {epoch:2d}/{EPOCHS}  loss={running_loss/len(train_ds):.4f}  val_acc={acc:.4f}  "
              f"({time.time()-t0:.1f}s)")

        if acc > best_acc:
            best_acc = acc
            torch.save(model.state_dict(), "sign_cnn_best.pt")

    print(f"\nbest val_acc: {best_acc:.4f}")
    model.load_state_dict(torch.load("sign_cnn_best.pt"))
    acc, per_c, per_t = evaluate(model, val_loader)
    print("per-class accuracy on fixed synthetic validation set:")
    for c in range(NUM_CLASSES):
        label = "none" if c == 0 else f"stage {c}"
        n = int(per_t[c])
        a = per_c[c] / n if n else float("nan")
        print(f"  class {c:2d} ({label:9s}): {a:.3f}  (n={n})")

    export_onnx(model)


def export_onnx(model):
    model.eval().to("cpu")
    dummy = torch.randn(1, 3, IMG_SIZE, IMG_SIZE)
    torch.onnx.export(
        model, dummy, "sign_cnn.onnx",
        input_names=["input"], output_names=["logits"],
        dynamic_axes={"input": {0: "batch"}, "logits": {0: "batch"}},
        opset_version=18,  # torch 2.10's exporter natively targets 18; a lower opset_version here
                           # triggers an internal downgrade pass that fails loudly (but non-fatally)
    )
    print("exported sign_cnn.onnx")

    import onnxruntime as ort
    sess = ort.InferenceSession("sign_cnn.onnx", providers=["CPUExecutionProvider"])
    with torch.no_grad():
        torch_out = model(dummy).numpy()
    onnx_out = sess.run(None, {"input": dummy.numpy()})[0]
    max_diff = np.abs(torch_out - onnx_out).max()
    print(f"ONNX vs PyTorch max output diff: {max_diff:.6f} (should be ~1e-5)")
    assert max_diff < 1e-3, "ONNX export mismatch - do not deploy"


if __name__ == "__main__":
    main()
