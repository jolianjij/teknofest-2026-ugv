#!/usr/bin/env python3
"""
Turns captured frames into a training set by auto-labelling them with the
classical heuristic - no manual annotation.

    python make_pseudo_labels.py --frames frames/ --out dataset/

Writes the layout train.py expects:
    dataset/images/*.jpg
    dataset/masks/*.png      0 = not drivable, 255 = drivable

Also writes dataset/preview/*.jpg overlays. LOOK AT THOSE before training:
the CNN can only ever learn to imitate these labels, so a systematically
wrong heuristic produces a confidently wrong model. Frames whose overlay is
clearly wrong should be deleted from dataset/images + dataset/masks before
training rather than left to teach the network nonsense.

Frames that come out almost entirely drivable or almost entirely not are
rejected automatically - both indicate the seed-region assumption failed for
that frame, and neither carries usable supervision.
"""
import argparse
import shutil
from pathlib import Path

import cv2

from ground_plane_heuristic import segment_drivable, overlay_preview

MIN_DRIVABLE_FRAC = 0.05
MAX_DRIVABLE_FRAC = 0.85


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", default="frames")
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--preview-every", type=int, default=10)
    args = ap.parse_args()

    frames_dir = Path(args.frames)
    out = Path(args.out)
    for sub in ("images", "masks", "preview"):
        (out / sub).mkdir(parents=True, exist_ok=True)

    files = sorted(list(frames_dir.glob("*.jpg")) + list(frames_dir.glob("*.png")))
    if not files:
        raise SystemExit(f"no frames found in {frames_dir}")

    kept = skipped = 0
    for i, path in enumerate(files):
        img = cv2.imread(str(path))
        if img is None:
            skipped += 1
            continue
        mask = segment_drivable(img)
        frac = (mask > 0).mean()
        if not (MIN_DRIVABLE_FRAC <= frac <= MAX_DRIVABLE_FRAC):
            skipped += 1
            continue

        stem = path.stem
        shutil.copy(path, out / "images" / f"{stem}.jpg")
        cv2.imwrite(str(out / "masks" / f"{stem}.png"), mask)
        if i % args.preview_every == 0:
            cv2.imwrite(str(out / "preview" / f"{stem}.jpg"), overlay_preview(img, mask))
        kept += 1

    print(f"kept {kept}, skipped {skipped} (implausible drivable fraction)")
    print(f"-> {out}/images, {out}/masks")
    print(f"INSPECT {out}/preview BEFORE TRAINING - the model can only imitate these labels.")


if __name__ == "__main__":
    main()
