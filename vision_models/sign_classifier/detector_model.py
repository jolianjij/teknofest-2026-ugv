"""
Single-object sign detector: one CNN does localization (bbox regression)
AND classification directly from pixels - no color thresholding anywhere,
per explicit request that detection itself must not depend on color.

Simplified relative to a general-purpose detector (YOLO-style grid/anchors)
because the task only ever has at most one relevant sign in frame at a
time (matches the actual competition setup - one stage sign visible as
the vehicle approaches it): the network predicts a single objectness
score, one bbox (cx, cy, w, h, normalized 0-1), and one 10-way class
distribution (stages 1-10) via global pooling, not a spatial grid.
"""
import torch
import torch.nn as nn

INPUT_SIZE = 128
NUM_STAGE_CLASSES = 10


class SignDetectorNet(nn.Module):
    def __init__(self):
        super().__init__()
        def block(cin, cout, stride=2):
            return nn.Sequential(
                nn.Conv2d(cin, cout, 3, stride=stride, padding=1), nn.BatchNorm2d(cout), nn.ReLU(inplace=True),
            )
        self.backbone = nn.Sequential(
            block(3, 16),     # 128 -> 64
            block(16, 32),    # 64 -> 32
            block(32, 64),    # 32 -> 16
            block(64, 96),    # 16 -> 8
            block(96, 128),   # 8 -> 4
        )
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.trunk = nn.Sequential(nn.Linear(128, 128), nn.ReLU(inplace=True))

        self.objectness_head = nn.Linear(128, 1)
        self.bbox_head = nn.Linear(128, 4)          # cx, cy, w, h (raw; sigmoid applied outside for cx/cy/w/h in [0,1])
        self.class_head = nn.Linear(128, NUM_STAGE_CLASSES)

    def forward(self, x):
        feat = self.backbone(x)
        feat = self.pool(feat).flatten(1)
        feat = self.trunk(feat)
        objectness = self.objectness_head(feat).squeeze(-1)   # raw logit
        bbox = torch.sigmoid(self.bbox_head(feat))             # (cx,cy,w,h) in [0,1]
        class_logits = self.class_head(feat)
        return objectness, bbox, class_logits
