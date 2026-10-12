"""
Lightweight binary (drivable / not-drivable) semantic segmentation model:
MobileNetV3-Small encoder (ImageNet-pretrained, transfer learning) + a
small LR-ASPP-style decoder. Small enough to run on the Orange Pi 5's
RK3588 NPU via RKNN conversion, matching CLAUDE.md's existing vision
strategy ("Front camera: onboard-NPU (RKNN) segmentation for drivable
terrain feeding the Nav2 semantic costmap").

NOT TRAINED YET. Unlike the sign classifier (vision_models/sign_classifier),
terrain appearance can't be procedurally synthesized convincingly - there
is no shortcut around needing real front-camera frames of the actual
parkour/similar off-road terrain. train.py is ready to consume
(image, mask) pairs the moment such frames exist (either hand-labeled, or
auto-labeled via ground_plane_heuristic.py's pseudo-labeler - see that
file's header for why this pattern is standard for terrain classification).
Do not deploy a checkpoint from this pipeline against real navigation
until it has been validated on real footage, per CLAUDE.md's working
agreement on hardware-dependent claims.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as tvm

NUM_CLASSES = 2  # 0 = not drivable, 1 = drivable
INPUT_SIZE = 256  # square input; real deployment should match the actual camera aspect ratio once chosen


class DrivableSegNet(nn.Module):
    def __init__(self, pretrained=True):
        super().__init__()
        weights = tvm.MobileNet_V3_Small_Weights.IMAGENET1K_V1 if pretrained else None
        backbone = tvm.mobilenet_v3_small(weights=weights).features

        # MobileNetV3-Small feature indices: low-level features at index 3
        # (24 channels, stride 8) and high-level at the final index (576
        # channels, stride 32) - an LR-ASPP-style two-tap decoder.
        self.low_level_idx = 3
        self.backbone = backbone

        low_channels = 24
        high_channels = 576

        self.aspp = nn.Sequential(
            nn.Conv2d(high_channels, 128, 1), nn.BatchNorm2d(128), nn.ReLU(inplace=True),
        )
        self.low_proj = nn.Sequential(
            nn.Conv2d(low_channels, 32, 1), nn.BatchNorm2d(32), nn.ReLU(inplace=True),
        )
        self.classifier = nn.Sequential(
            nn.Conv2d(128 + 32, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(inplace=True),
            nn.Conv2d(64, NUM_CLASSES, 1),
        )

    def forward(self, x):
        input_size = x.shape[-2:]
        low_feat = None
        for i, layer in enumerate(self.backbone):
            x = layer(x)
            if i == self.low_level_idx:
                low_feat = x
        high_feat = x

        high_feat = self.aspp(high_feat)
        high_feat = F.interpolate(high_feat, size=low_feat.shape[-2:], mode="bilinear", align_corners=False)
        low_feat = self.low_proj(low_feat)

        merged = torch.cat([high_feat, low_feat], dim=1)
        out = self.classifier(merged)
        out = F.interpolate(out, size=input_size, mode="bilinear", align_corners=False)
        return out
