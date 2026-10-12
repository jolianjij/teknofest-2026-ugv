"""
Trains SignDetectorNet (localization + classification in one model, no
color thresholding) on synthetic scenes from dataset_detector.py.

Run with the IOAI2026 venv:
    C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe train_detector.py
"""
import gc
import sys
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader

sys.path.insert(0, ".")
from dataset_detector import generate_detection_sample, IMG_SIZE
from detector_model import SignDetectorNet

SAMPLES_PER_EPOCH = 8000
VAL_SAMPLES = 1000
BATCH_SIZE = 128
EPOCHS = 25
LR = 1e-3
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

MEAN = np.array([0.5, 0.5, 0.5], dtype=np.float32)
STD = np.array([0.25, 0.25, 0.25], dtype=np.float32)


class DetectionDataset(Dataset):
    def __init__(self, length, seed_base=None):
        self.length = length
        self.seed_base = seed_base

    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        rng = (self.seed_base + idx) if self.seed_base is not None else None
        img, has_obj, cls, bbox = generate_detection_sample(rng)
        img = (img.astype(np.float32) / 255.0 - MEAN) / STD
        img = np.transpose(img, (2, 0, 1))
        cls_idx = (cls - 1) if has_obj else -1  # -1 = ignore (masked out of the class loss)
        return (torch.from_numpy(img), torch.tensor(float(has_obj)),
                torch.tensor(bbox, dtype=torch.float32), torch.tensor(cls_idx, dtype=torch.long))


def iou_xywh(pred, target):
    def to_xyxy(b):
        cx, cy, w, h = b.unbind(-1)
        return cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
    px0, py0, px1, py1 = to_xyxy(pred)
    tx0, ty0, tx1, ty1 = to_xyxy(target)
    ix0, iy0 = torch.max(px0, tx0), torch.max(py0, ty0)
    ix1, iy1 = torch.min(px1, tx1), torch.min(py1, ty1)
    inter = (ix1 - ix0).clamp(min=0) * (iy1 - iy0).clamp(min=0)
    area_p = (px1 - px0).clamp(min=0) * (py1 - py0).clamp(min=0)
    area_t = (tx1 - tx0).clamp(min=0) * (ty1 - ty0).clamp(min=0)
    union = area_p + area_t - inter
    return inter / union.clamp(min=1e-6)


def compute_loss(objectness, bbox, class_logits, has_obj, target_bbox, target_cls):
    obj_loss = F.binary_cross_entropy_with_logits(objectness, has_obj)

    pos_mask = has_obj > 0.5
    if pos_mask.any():
        bbox_loss = F.mse_loss(bbox[pos_mask], target_bbox[pos_mask])
        cls_loss = F.cross_entropy(class_logits[pos_mask], target_cls[pos_mask])
    else:
        bbox_loss = torch.tensor(0.0, device=objectness.device)
        cls_loss = torch.tensor(0.0, device=objectness.device)

    return obj_loss + 3.0 * bbox_loss + cls_loss, obj_loss.item(), bbox_loss.item(), cls_loss.item()


def evaluate(model, loader):
    model.eval()
    obj_correct, obj_total = 0, 0
    cls_correct, cls_total = 0, 0
    ious = []
    with torch.no_grad():
        for img, has_obj, bbox, cls in loader:
            img, has_obj, bbox, cls = img.to(DEVICE), has_obj.to(DEVICE), bbox.to(DEVICE), cls.to(DEVICE)
            objectness, pred_bbox, class_logits = model(img)
            pred_obj = (torch.sigmoid(objectness) > 0.5).float()
            obj_correct += (pred_obj == has_obj).sum().item()
            obj_total += has_obj.size(0)

            pos_mask = has_obj > 0.5
            if pos_mask.any():
                pred_cls = class_logits[pos_mask].argmax(1)
                cls_correct += (pred_cls == cls[pos_mask]).sum().item()
                cls_total += pos_mask.sum().item()
                ious.append(iou_xywh(pred_bbox[pos_mask], bbox[pos_mask]).mean().item())
    model.train()
    mean_iou = float(np.mean(ious)) if ious else 0.0
    return obj_correct / obj_total, (cls_correct / cls_total if cls_total else 0.0), mean_iou


def main():
    print(f"device: {DEVICE}", flush=True)
    train_loader = DataLoader(DetectionDataset(SAMPLES_PER_EPOCH), batch_size=BATCH_SIZE, shuffle=True, num_workers=0)
    val_loader = DataLoader(DetectionDataset(VAL_SAMPLES, seed_base=999_000), batch_size=BATCH_SIZE,
                             shuffle=False, num_workers=0)

    model = SignDetectorNet().to(DEVICE)
    opt = torch.optim.Adam(model.parameters(), lr=LR)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=EPOCHS)

    best_score = -1.0
    for epoch in range(1, EPOCHS + 1):
        t0 = time.time()
        running = 0.0
        for img, has_obj, bbox, cls in train_loader:
            img, has_obj, bbox, cls = img.to(DEVICE), has_obj.to(DEVICE), bbox.to(DEVICE), cls.to(DEVICE)
            opt.zero_grad()
            objectness, pred_bbox, class_logits = model(img)
            loss, ol, bl, cl = compute_loss(objectness, pred_bbox, class_logits, has_obj, bbox, cls)
            loss.backward()
            opt.step()
            running += loss.item() * img.size(0)
        sched.step()

        obj_acc, cls_acc, mean_iou = evaluate(model, val_loader)
        score = obj_acc + cls_acc + mean_iou
        print(f"epoch {epoch:2d}/{EPOCHS}  loss={running/SAMPLES_PER_EPOCH:.4f}  "
              f"obj_acc={obj_acc:.4f}  cls_acc={cls_acc:.4f}  mean_iou={mean_iou:.4f}  ({time.time()-t0:.1f}s)",
              flush=True)

        if score > best_score:
            best_score = score
            torch.save(model.state_dict(), "sign_detector_best.pt")

        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    print(f"\nbest combined score: {best_score:.4f}")
    export_onnx()


def export_onnx():
    model = SignDetectorNet()
    model.load_state_dict(torch.load("sign_detector_best.pt", map_location="cpu"))
    model.eval()
    dummy = torch.randn(1, 3, IMG_SIZE, IMG_SIZE)
    torch.onnx.export(
        model, dummy, "sign_detector.onnx",
        input_names=["input"], output_names=["objectness", "bbox", "class_logits"],
        dynamic_axes={"input": {0: "batch"}},
        opset_version=18,
    )
    print("exported sign_detector.onnx")

    import onnxruntime as ort
    sess = ort.InferenceSession("sign_detector.onnx", providers=["CPUExecutionProvider"])
    with torch.no_grad():
        t_obj, t_bbox, t_cls = model(dummy)
    o_obj, o_bbox, o_cls = sess.run(None, {"input": dummy.numpy()})
    diff = max(np.abs(t_obj.numpy() - o_obj).max(), np.abs(t_bbox.numpy() - o_bbox).max(),
               np.abs(t_cls.numpy() - o_cls).max())
    print(f"ONNX vs PyTorch max diff: {diff:.6f}")


if __name__ == "__main__":
    main()
