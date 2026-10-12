#!/usr/bin/env python3
"""
Converts drivable_segnet.onnx to a .rknn model for the Orange Pi 5's NPU.

WHY: segmentation currently runs on the Pi's CPU via onnxruntime and costs a
full core even throttled to every 5th frame - it was measured saturating four
cores at full rate and starving the camera capture loop down to 5 fps. The
RK3588 has a ~7 TOPS NPU sitting completely idle. Moving inference there frees
the CPU for everything else and lets segmentation run at a useful rate.

WHERE THIS RUNS: rknn-toolkit2 is x86_64-Linux only. It cannot run on Windows
and it cannot run on the Pi (the Pi gets rknn-toolkit-lite2, which only
*executes* .rknn files, it cannot build them). Run this inside WSL:

    wsl -d Ubuntu-22.04
    pip3 install rknn-toolkit2
    python3 convert_to_rknn.py --onnx drivable_segnet.onnx --out drivable_segnet.rknn

Then copy the .rknn to ~/teknofest_ws/ on the Pi. path_segmentation_node.py
prefers it automatically and falls back to ONNX-on-CPU if it is absent.

QUANTISATION: left OFF by default. int8 quantisation needs a representative
calibration set drawn from the real deployment distribution, and the only
frames available were captured indoors - quantising against those would bake
in the wrong statistics. Enable --quantize once real course footage exists and
verify IoU against the float model before trusting it.
"""
import argparse
import sys

TARGET_PLATFORM = "rk3588"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", default="drivable_segnet.onnx")
    ap.add_argument("--out", default="drivable_segnet.rknn")
    ap.add_argument("--quantize", action="store_true",
                    help="int8 quantisation; requires --dataset")
    ap.add_argument("--dataset", default="",
                    help="text file listing calibration images, one path per line")
    args = ap.parse_args()

    try:
        from rknn.api import RKNN
    except ImportError:
        sys.exit(
            "rknn-toolkit2 not found. It is x86_64-Linux only - run this in WSL:\n"
            "    wsl -d Ubuntu-22.04\n"
            "    pip3 install rknn-toolkit2"
        )

    if args.quantize and not args.dataset:
        sys.exit("--quantize requires --dataset (calibration images from the REAL course)")

    rknn = RKNN(verbose=False)

    # Normalisation must match train.py / the node exactly: ImageNet mean and
    # std, expressed here in 0-255 units because RKNN folds this into the graph
    # and is then fed raw uint8 frames.
    rknn.config(
        mean_values=[[0.485 * 255, 0.456 * 255, 0.406 * 255]],
        std_values=[[0.229 * 255, 0.224 * 255, 0.225 * 255]],
        target_platform=TARGET_PLATFORM,
    )

    print(f"loading {args.onnx}")
    if rknn.load_onnx(model=args.onnx) != 0:
        sys.exit("load_onnx failed")

    print(f"building (quantize={args.quantize})")
    ret = rknn.build(do_quantization=args.quantize,
                     dataset=args.dataset if args.quantize else None)
    if ret != 0:
        sys.exit("build failed")

    if rknn.export_rknn(args.out) != 0:
        sys.exit("export_rknn failed")

    print(f"wrote {args.out}")
    print("copy it to ~/teknofest_ws/ on the Pi; the node prefers .rknn automatically")
    rknn.release()


if __name__ == "__main__":
    main()
