"""
Interactive webcam capture tool for real training data - closes the
sim-to-real gap the synthetic-only YOLO model failed on (confirmed: never
detects real signs, false-positives on random objects, wrong class when
it does trigger something). Captures real frames with a fixed on-screen
guide box, labeled instantly by keypress - no manual bounding-box
annotation needed.

Print printable_signs.pdf first. Then, for each stage sign:
  - Hold the printed sign up so it fills the yellow guide box on screen.
  - Press the matching digit key (1-9 for stages 1-9, 0 for stage 10) to
    capture+label that frame as that stage.
  - Vary distance, angle/tilt, and position slightly between captures -
    press the same key repeatedly (aim for ~25-40 captures per stage).
  - Also move around the room/lighting between captures if possible
    (different backgrounds, near a window vs. under a lamp, etc.) - this
    variety is what actually teaches the model to generalize.

For negatives (no sign - directly targets the false-positive problem):
  - Point the camera at ordinary room objects, your face, red/round
    things that aren't the sign, etc.
  - Press 'n' to capture+label as background/no-object.
  - Aim for at least as many negatives as positives combined, prioritizing
    anything that previously triggered a false positive.

Press 'q' to quit. Frames save immediately to dataset_real/images/ and
dataset_real/labels/ in YOLO format, ready for finetune_real.py.

Run with the IOAI2026 venv: C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe capture_real_data.py
"""
import time
from pathlib import Path

import cv2

OUT_DIR = Path("dataset_real")
GUIDE_BOX_FRAC = 0.5  # guide box side length as a fraction of the shorter frame dimension


def main():
    img_dir = OUT_DIR / "images"
    lbl_dir = OUT_DIR / "labels"
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    existing = list(img_dir.glob("*.jpg"))
    counter = len(existing)
    counts_by_class = {}

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("could not open webcam (index 0)")
        return

    print("running - hold up a printed sign inside the yellow box, press its digit key to capture")
    print("press 'n' for a no-sign negative frame, 'q' to quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("frame grab failed")
            break

        h, w = frame.shape[:2]
        side = int(min(h, w) * GUIDE_BOX_FRAC)
        x0, y0 = (w - side) // 2, (h - side) // 2
        x1, y1 = x0 + side, y0 + side

        vis = frame.copy()
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 255), 2)
        cv2.putText(vis, f"captured: {counter}  classes: {counts_by_class}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.putText(vis, "digit=capture stage | n=negative | q=quit", (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

        cv2.imshow("real data capture", vis)
        key = cv2.waitKey(1) & 0xFF

        label = None
        if key == ord('q'):
            break
        elif key == ord('n'):
            label = None  # negative: empty label file, no class
        elif ord('0') <= key <= ord('9'):
            digit = key - ord('0')
            stage = 10 if digit == 0 else digit
            label = stage - 1  # 0-indexed for YOLO, matches export_yolo_dataset.py's convention
        else:
            continue

        ts = int(time.time() * 1000)
        img_path = img_dir / f"real_{ts}_{counter:05d}.jpg"
        lbl_path = lbl_dir / f"real_{ts}_{counter:05d}.txt"
        cv2.imwrite(str(img_path), frame)

        if label is None:
            lbl_path.write_text("")
            counts_by_class['neg'] = counts_by_class.get('neg', 0) + 1
        else:
            cx = (x0 + x1) / 2 / w
            cy = (y0 + y1) / 2 / h
            bw = side / w
            bh = side / h
            lbl_path.write_text(f"{label} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
            counts_by_class[label + 1] = counts_by_class.get(label + 1, 0) + 1

        counter += 1

    cap.release()
    cv2.destroyAllWindows()
    print(f"done - {counter} total frames in {img_dir}, breakdown: {counts_by_class}")


if __name__ == "__main__":
    main()
