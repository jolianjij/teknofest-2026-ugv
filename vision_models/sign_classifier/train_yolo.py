"""
Fine-tunes YOLOv8n (COCO-pretrained, via ultralytics) on the synthetic sign
dataset exported by export_yolo_dataset.py - replaces the from-scratch
single-object detector (detector_model.py/train_detector.py), whose bbox
IoU (~0.45) was too weak. YOLOv8n brings a real anchor-free detection head,
CIoU/DFL box loss, mosaic augmentation, and transfer learning from COCO
features - all things the hand-rolled model lacked.

"nano" size is deliberate: this still needs to run on the Orange Pi 5
eventually (CPU or RKNN/NPU), not just this laptop's GPU.

Run with the IOAI2026 venv (has ultralytics 8.4.84 already):
    C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe train_yolo.py
"""
from ultralytics import YOLO

DATA_YAML = "dataset_yolo/data.yaml"
IMGSZ = 320
EPOCHS = 60
BATCH = 32


def main():
    model = YOLO("yolov8n.pt")
    model.train(
        data=DATA_YAML,
        imgsz=IMGSZ,
        epochs=EPOCHS,
        batch=BATCH,
        project="runs",
        name="sign_yolo",
        patience=15,
        val=True,
    )
    metrics = model.val()
    print("final val metrics:", metrics.results_dict)

    model.export(format="onnx", imgsz=IMGSZ, opset=18)
    print("exported ONNX model under runs/sign_yolo/weights/")


if __name__ == "__main__":
    main()
