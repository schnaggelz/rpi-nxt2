#!/usr/bin/env python3
"""Run inference on a single image with a trained shapes model."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
from PIL import Image
from torch import nn


class SmallCNN(nn.Module):
    def __init__(self, num_classes: int):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.classifier = nn.Linear(64, num_classes)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.features(x)
        x = x.view(x.size(0), -1)
        return self.classifier(x)


def image_to_tensor(path: Path) -> torch.Tensor:
    image = Image.open(path).convert("RGB")
    w, h = image.size
    tensor = torch.ByteTensor(torch.ByteStorage.from_buffer(image.tobytes()))
    tensor = tensor.view(h, w, 3).permute(2, 0, 1).float() / 255.0
    return tensor.unsqueeze(0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=Path("outputs/model.pt"))
    parser.add_argument("--image", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    checkpoint = torch.load(args.model, map_location=device)
    class_to_idx = checkpoint["class_to_idx"]
    idx_to_class = {idx: name for name, idx in class_to_idx.items()}

    model = SmallCNN(num_classes=len(class_to_idx)).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    x = image_to_tensor(args.image).to(device)

    with torch.no_grad():
        logits = model(x)
        probs = torch.softmax(logits, dim=1)[0]
        pred_idx = int(probs.argmax().item())

    pred_name = idx_to_class[pred_idx]
    confidence = float(probs[pred_idx].item())

    print(f"Predicted class: {pred_name}")
    print(f"Confidence: {confidence:.3f}")


if __name__ == "__main__":
    main()
