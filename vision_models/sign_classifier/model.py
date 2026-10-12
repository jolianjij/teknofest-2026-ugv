"""Small CNN for classifying a cropped sign-candidate ROI into one of the
11 classes (0=not a sign, 1-10=stage number). Deliberately tiny - this runs
on the Orange Pi 5's CPU, not the NPU (NPU is reserved for terrain
segmentation per CLAUDE.md's vision strategy), and only ever sees a small
region proposed by the classical red-ring localizer in sign_detector.py,
not the full frame."""
import torch.nn as nn

NUM_CLASSES = 11


class SignCNN(nn.Module):
    def __init__(self, num_classes=NUM_CLASSES):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1), nn.BatchNorm2d(16), nn.ReLU(inplace=True), nn.MaxPool2d(2),   # 64->32
            nn.Conv2d(16, 32, 3, padding=1), nn.BatchNorm2d(32), nn.ReLU(inplace=True), nn.MaxPool2d(2),  # 32->16
            nn.Conv2d(32, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(inplace=True), nn.MaxPool2d(2),  # 16->8
            nn.Conv2d(64, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d(1),
        )
        self.classifier = nn.Linear(64, num_classes)

    def forward(self, x):
        x = self.features(x)
        x = x.flatten(1)
        return self.classifier(x)
