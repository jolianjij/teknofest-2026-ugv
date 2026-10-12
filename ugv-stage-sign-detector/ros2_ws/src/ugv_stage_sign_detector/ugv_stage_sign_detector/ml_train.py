"""Train, export, and evaluate the tiny 13-class sign CNN.

This module is a laptop-side development tool.  The exported ONNX file is the
only model artifact required by the Raspberry Pi runtime.
"""

from __future__ import annotations

import argparse
import csv
import json
import random as python_random
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from .classification import OnnxSignClassifier, preprocess_sign_patch
from .ml_dataset import CLASS_LABELS, class_name, iter_dataset_images
from .ml_model import TinySignCNN, require_torch, torch

try:
    from torch.utils.data import DataLoader, Dataset
except (ModuleNotFoundError, AttributeError):
    DataLoader = None
    Dataset = object


class SignImageDataset(Dataset):
    """Read one or more datasets sharing the documented split/class layout."""

    def __init__(self, roots: list[Path], split: str, input_size: int = 64) -> None:
        require_torch()
        self.samples = list(iter_dataset_images(roots, split))
        self.input_size = input_size
        if not self.samples:
            raise ValueError(f"No images found for split '{split}' in: {roots}")

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int):
        path, label = self.samples[index]
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise OSError(f"Could not read training image: {path}")
        array = preprocess_sign_patch(image, self.input_size)[0]
        return torch.from_numpy(array), int(label)


def train_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Train and export the UGV stage-sign CNN")
    parser.add_argument("dataset", help="Primary dataset root")
    parser.add_argument("--additional-root", action="append", default=[])
    parser.add_argument("--output", required=True, help="ONNX file to create")
    parser.add_argument("--epochs", type=int, default=25)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=0.001)
    parser.add_argument("--weight-decay", type=float, default=0.0001)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    args = parser.parse_args(argv)

    require_torch()
    if args.epochs < 1 or args.batch_size < 1 or args.workers < 0:
        parser.error("epochs and batch-size must be positive; workers cannot be negative")
    roots = [Path(args.dataset).expanduser().resolve()] + [
        Path(value).expanduser().resolve() for value in args.additional_root
    ]
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    _set_seed(args.seed)

    train_data = SignImageDataset(roots, "train")
    validation_data = SignImageDataset(roots, "val")
    test_data = SignImageDataset(roots, "test")
    device = _select_device(args.device)
    print(f"Training device: {device}", flush=True)
    print(
        f"Images: train={len(train_data)}, val={len(validation_data)}, test={len(test_data)}",
        flush=True,
    )

    generator = torch.Generator().manual_seed(args.seed)
    train_loader = DataLoader(
        train_data,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        generator=generator,
    )
    validation_loader = DataLoader(
        validation_data,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
    )
    test_loader = DataLoader(
        test_data,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
    )

    model = TinySignCNN(class_count=len(CLASS_LABELS)).to(device)
    class_weights = _class_weights(train_data).to(device)
    criterion = torch.nn.CrossEntropyLoss(weight=class_weights, label_smoothing=0.05)
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer,
        mode="max",
        factor=0.5,
        patience=3,
    )

    checkpoint = output.with_suffix(".pt")
    best_accuracy = -1.0
    history: list[dict[str, float | int]] = []
    for epoch in range(1, args.epochs + 1):
        model.train()
        training_loss = 0.0
        training_count = 0
        for inputs, labels in train_loader:
            inputs = inputs.to(device)
            labels = labels.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits = model(inputs)
            loss = criterion(logits, labels)
            loss.backward()
            optimizer.step()
            training_loss += float(loss.item()) * labels.shape[0]
            training_count += int(labels.shape[0])

        validation = _evaluate_torch(model, validation_loader, criterion, device)
        average_training_loss = training_loss / max(1, training_count)
        scheduler.step(validation["accuracy"])
        current_lr = float(optimizer.param_groups[0]["lr"])
        row = {
            "epoch": epoch,
            "train_loss": average_training_loss,
            "val_loss": float(validation["loss"]),
            "val_accuracy": float(validation["accuracy"]),
            "learning_rate": current_lr,
        }
        history.append(row)
        print(
            f"epoch {epoch:02d}/{args.epochs}: train_loss={average_training_loss:.4f} "
            f"val_loss={validation['loss']:.4f} val_accuracy={validation['accuracy']:.4f}",
            flush=True,
        )
        if float(validation["accuracy"]) > best_accuracy:
            best_accuracy = float(validation["accuracy"])
            torch.save(
                {
                    "model_state": model.state_dict(),
                    "labels": list(CLASS_LABELS),
                    "input_size": 64,
                    "epoch": epoch,
                    "validation_accuracy": best_accuracy,
                },
                checkpoint,
            )

    saved = torch.load(checkpoint, map_location=device, weights_only=True)
    model.load_state_dict(saved["model_state"])
    model.eval()
    test_metrics = _evaluate_torch(model, test_loader, criterion, device)
    _export_onnx(model, output, device)
    _verify_onnx(output)

    history_path = output.with_name(output.stem + "_history.csv")
    with history_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(history[0]))
        writer.writeheader()
        writer.writerows(history)

    metrics_path = output.with_name(output.stem + "_metrics.json")
    metrics = {
        "labels": list(CLASS_LABELS),
        "class_names": [class_name(label) for label in CLASS_LABELS],
        "best_validation_accuracy": best_accuracy,
        "test_accuracy": test_metrics["accuracy"],
        "test_loss": test_metrics["loss"],
        "test_per_class_accuracy": test_metrics["per_class_accuracy"],
        "test_confusion_matrix": test_metrics["confusion_matrix"],
        "dataset_roots": [str(root) for root in roots],
        "seed": args.seed,
    }
    metrics_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    print(f"Best checkpoint: {checkpoint}", flush=True)
    print(f"ONNX runtime model: {output}", flush=True)
    print(f"Test accuracy: {test_metrics['accuracy']:.4f}", flush=True)
    print(f"Metrics: {metrics_path}", flush=True)
    return 0


def evaluate_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Evaluate an ONNX sign classifier")
    parser.add_argument("model", help="ONNX model")
    parser.add_argument("dataset", help="Dataset root")
    parser.add_argument("--additional-root", action="append", default=[])
    parser.add_argument("--split", choices=("train", "val", "test"), default="test")
    parser.add_argument("--output", help="Optional JSON metrics path")
    args = parser.parse_args(argv)

    roots = [Path(args.dataset).expanduser().resolve()] + [
        Path(value).expanduser().resolve() for value in args.additional_root
    ]
    classifier = OnnxSignClassifier(args.model, list(CLASS_LABELS), input_size=64)
    confusion = np.zeros((len(CLASS_LABELS), len(CLASS_LABELS)), dtype=np.int64)
    total = 0
    correct = 0
    for path, expected in iter_dataset_images(roots, args.split):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise OSError(f"Could not read evaluation image: {path}")
        result = classifier.classify(image)
        predicted = int(result.label)
        confusion[expected, predicted] += 1
        total += 1
        correct += int(predicted == expected)
    if total == 0:
        raise ValueError(f"No {args.split} images were found in: {roots}")

    metrics = _metrics_from_confusion(confusion)
    metrics["split"] = args.split
    metrics["total"] = total
    metrics["accuracy"] = correct / total
    print(f"ONNX {args.split} accuracy: {metrics['accuracy']:.4f} ({correct}/{total})")
    print("Per-class accuracy:")
    for label, accuracy in metrics["per_class_accuracy"].items():
        print(f"  {label:>7}: {accuracy:.4f}")
    if args.output:
        destination = Path(args.output).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        print(f"Saved metrics: {destination}")
    return 0


def _set_seed(seed: int) -> None:
    python_random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _select_device(requested: str):
    if requested == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")
    if requested == "cuda" or (requested == "auto" and torch.cuda.is_available()):
        return torch.device("cuda")
    return torch.device("cpu")


def _class_weights(dataset: SignImageDataset):
    counts = Counter(label for _, label in dataset.samples)
    if any(counts.get(label, 0) == 0 for label in CLASS_LABELS):
        missing = [class_name(label) for label in CLASS_LABELS if counts.get(label, 0) == 0]
        raise ValueError(f"Training split is missing classes: {missing}")
    maximum = max(counts.values())
    return torch.tensor(
        [maximum / counts[label] for label in CLASS_LABELS],
        dtype=torch.float32,
    )


def _evaluate_torch(model, loader, criterion, device) -> dict:
    model.eval()
    total_loss = 0.0
    total = 0
    confusion = np.zeros((len(CLASS_LABELS), len(CLASS_LABELS)), dtype=np.int64)
    with torch.inference_mode():
        for inputs, labels in loader:
            inputs = inputs.to(device)
            labels = labels.to(device)
            logits = model(inputs)
            loss = criterion(logits, labels)
            predictions = torch.argmax(logits, dim=1)
            total_loss += float(loss.item()) * labels.shape[0]
            total += int(labels.shape[0])
            for expected, predicted in zip(
                labels.detach().cpu().tolist(),
                predictions.detach().cpu().tolist(),
            ):
                confusion[int(expected), int(predicted)] += 1
    metrics = _metrics_from_confusion(confusion)
    metrics["loss"] = total_loss / max(1, total)
    return metrics


def _metrics_from_confusion(confusion: np.ndarray) -> dict:
    total = int(confusion.sum())
    correct = int(np.trace(confusion))
    per_class = {}
    for label in CLASS_LABELS:
        count = int(confusion[label].sum())
        per_class[class_name(label)] = (
            float(confusion[label, label] / count) if count else 0.0
        )
    return {
        "accuracy": correct / max(1, total),
        "per_class_accuracy": per_class,
        "confusion_matrix": confusion.tolist(),
    }


def _export_onnx(model, output: Path, device) -> None:
    dummy = torch.zeros((1, 1, 64, 64), dtype=torch.float32, device=device)
    torch.onnx.export(
        model,
        dummy,
        str(output),
        input_names=["image"],
        output_names=["logits"],
        dynamic_axes={"image": {0: "batch"}, "logits": {0: "batch"}},
        opset_version=17,
        do_constant_folding=True,
        dynamo=False,
    )


def _verify_onnx(path: Path) -> None:
    # OpenCV is the production runtime, so export is not complete until the
    # produced model can be loaded and executed by OpenCV DNN.
    classifier = OnnxSignClassifier(path, list(CLASS_LABELS), input_size=64)
    blank = np.full((160, 160, 3), 255, dtype=np.uint8)
    classifier.classify(blank)


if __name__ == "__main__":
    raise SystemExit(train_main())
