"""
Live test of the fine-tuned YOLOv8n sign detector against a webcam - this
replaces the from-scratch single-object detector (detector_model.py),
whose bbox IoU (~0.45) was too weak per explicit feedback. Fine-tuned on
the same synthetic dataset (exported to disk by export_yolo_dataset.py)
but with a real detector architecture (anchor-free head, CIoU/DFL box
loss, mosaic augmentation, COCO-pretrained backbone) - final synthetic
validation: precision=0.994, recall=0.996, mAP50=0.995, mAP50-95=0.994.

Still synthetic-only training - no real printed sign exists yet, so this
is a smoke test of the pipeline, not a real-world accuracy check. Point
the camera at a monitor/phone showing one of the rendered training
samples (see detector_preview2.png) to exercise classification; an
ordinary room should mostly show no detections.

Run with the IOAI2026 venv: C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe yolo_live_test.py
Press 'q' to quit.
"""
import cv2
from ultralytics import YOLO

MODEL_PATH = "runs/detect/runs/sign_yolo/weights/best.pt"
CONF_THRESHOLD = 0.5


def main():
    model = YOLO(MODEL_PATH)

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("could not open webcam (index 0)")
        return

    print("running - press 'q' in the video window to quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("frame grab failed")
            break

        results = model.predict(frame, conf=CONF_THRESHOLD, imgsz=320, verbose=False)
        vis = results[0].plot()  # ultralytics' own bbox/label/confidence overlay

        cv2.imshow("YOLO sign detector live test", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
