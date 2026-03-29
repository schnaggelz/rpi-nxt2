#!/usr/bin/env python3
"""Export a trained SmallCNN checkpoint to ONNX."""

from __future__ import annotations

import argparse
from pathlib import Path

DEFAULT_MODEL = "outputs/model.pt"
DEFAULT_OUT_DIR = "outputs"


def export_onnx(model_path: Path, out_path: Path, image_size: int) -> Path:
    try:
        import torch
        import torch.nn as nn
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "PyTorch is required for ONNX export. Use the PyTorch venv."
        ) from exc

    # Must match the model used in train_classifier.py.
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

        def forward(self, x):
            x = self.features(x)
            x = x.view(x.size(0), -1)
            return self.classifier(x)

    checkpoint = torch.load(model_path, map_location="cpu")
    class_to_idx: dict[str, int] = checkpoint["class_to_idx"]
    num_classes = len(class_to_idx)

    model = SmallCNN(num_classes=num_classes)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    dummy = torch.zeros(1, 3, image_size, image_size)
    onnx_path = out_path / "model.onnx"

    try:
        torch.onnx.export(
            model,
            dummy,
            str(onnx_path),
            input_names=["input"],
            output_names=["output"],
            dynamic_axes={"input": {0: "batch"}, "output": {0: "batch"}},
            opset_version=11,
        )
    except ModuleNotFoundError as exc:
        if exc.name == "onnxscript":
            raise RuntimeError(
                "ONNX export requires onnxscript in the PyTorch venv. Install it with: pip install onnxscript"
            ) from exc
        raise

    print(f"ONNX model saved: {onnx_path}")
    return onnx_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export SmallCNN checkpoint to ONNX.")
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        help="Path to the trained model.pt checkpoint.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="Output directory for model.onnx.",
    )
    parser.add_argument(
        "--image-size",
        type=int,
        default=64,
        help="Square input resolution used in training. Default: 64",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    export_onnx(args.model, args.out_dir, args.image_size)


if __name__ == "__main__":
    main()
