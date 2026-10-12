"""PyTorch definition of the compact sign classifier used only for training."""

from __future__ import annotations

try:
    import torch
    from torch import nn
except ModuleNotFoundError:  # Runtime on the Pi deliberately omits PyTorch.
    torch = None
    nn = None


def require_torch() -> None:
    if torch is None or nn is None:
        raise RuntimeError(
            "ML training dependencies are not installed. Run "
            "'python -m pip install -r requirements-ml.txt' in the Windows venv."
        )


class TinySignCNN(nn.Module if nn is not None else object):
    """A compact grayscale CNN sized for crop-only Raspberry Pi inference."""

    def __init__(self, class_count: int = 13) -> None:
        require_torch()
        super().__init__()
        self.features = nn.Sequential(
            _conv_block(1, 16),
            nn.MaxPool2d(2),
            _conv_block(16, 32),
            nn.MaxPool2d(2),
            _conv_block(32, 64),
            nn.MaxPool2d(2),
            _conv_block(64, 96),
            nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(96 * 4 * 4, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.20),
            nn.Linear(128, class_count),
        )

    def forward(self, inputs):
        return self.classifier(self.features(inputs))


def _conv_block(input_channels: int, output_channels: int):
    require_torch()
    return nn.Sequential(
        nn.Conv2d(input_channels, output_channels, kernel_size=3, padding=1, bias=False),
        nn.BatchNorm2d(output_channels),
        nn.ReLU(inplace=True),
    )
