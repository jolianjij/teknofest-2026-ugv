"""
Live test of sign_detector.onnx (see train_detector.py) against a webcam -
detection AND classification from one model, no color thresholding.

This is a quick local sanity check on the laptop's webcam, NOT the same as
validating against the Orange Pi's actual front camera. It also can't
represent the real competition scene (no sign has been printed yet) - the
practical way to exercise it right now is to point the webcam at a stage
sign rendered full-screen on a monitor/phone, or just confirm it correctly
reports "no sign" against an ordinary room.

Run with the IOAI2026 venv (needs a GUI session for cv2.imshow, not
plain SSH/headless): C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe live_test.py
Press 'q' to quit.
"""
import sys
import time

import cv2
import numpy as np
import onnxruntime as ort

sys.path.insert(0, ".")
from detector_model import INPUT_SIZE, NUM_STAGE_CLASSES

MODEL_PATH = "sign_detector.onnx"
OBJECTNESS_THRESHOLD = 0.5
MEAN = np.array([0.5, 0.5, 0.5], dtype=np.float32)
STD = np.array([0.25, 0.25, 0.25], dtype=np.float32)


def preprocess(frame_bgr):
    h, w = frame_bgr.shape[:2]
    side = min(h, w)
    y0, x0 = (h - side) // 2, (w - side) // 2
    square = frame_bgr[y0:y0 + side, x0:x0 + side]
    rgb = cv2.cvtColor(square, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_LINEAR)
    arr = (resized.astype(np.float32) / 255.0 - MEAN) / STD
    arr = np.transpose(arr, (2, 0, 1))[None, ...]
    return arr, (x0, y0, side)


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def main():
    sess = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("could not open webcam (index 0)")
        return

    print("running - press 'q' in the video window to quit")
    fps_t0 = time.time()
    frame_count = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("frame grab failed")
            break

        inp, (x0, y0, side) = preprocess(frame)
        objectness, bbox, class_logits = sess.run(None, {"input": inp})
        obj_conf = float(sigmoid(objectness[0]))
        cx, cy, w, h = bbox[0]
        cls_probs = np.exp(class_logits[0] - class_logits[0].max())
        cls_probs /= cls_probs.sum()
        cls_idx = int(cls_probs.argmax())
        cls_conf = float(cls_probs[cls_idx])
        stage = cls_idx + 1  # class head is 0-indexed for stages 1-10

        vis = frame.copy()
        cv2.rectangle(vis, (x0, y0), (x0 + side, y0 + side), (80, 80, 80), 1)  # the square crop actually fed to the model

        if obj_conf > OBJECTNESS_THRESHOLD:
            bx0 = int(x0 + (cx - w / 2) * side)
            by0 = int(y0 + (cy - h / 2) * side)
            bx1 = int(x0 + (cx + w / 2) * side)
            by1 = int(y0 + (cy + h / 2) * side)
            cv2.rectangle(vis, (bx0, by0), (bx1, by1), (0, 255, 0), 2)
            label = f"stage {stage}  obj={obj_conf:.2f} cls={cls_conf:.2f}"
            cv2.putText(vis, label, (bx0, max(20, by0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(vis, f"no sign (obj={obj_conf:.2f})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        frame_count += 1
        if frame_count % 15 == 0:
            fps = 15 / (time.time() - fps_t0)
            fps_t0 = time.time()
            cv2.putText(vis, f"{fps:.1f} fps", (10, vis.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        cv2.imshow("sign_detector live test", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
