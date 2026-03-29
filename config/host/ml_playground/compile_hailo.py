#!/usr/bin/env python3
"""Compile a trained SmallCNN to a Hailo HEF file.

Pipeline:
  model.pt  ──►  model.onnx  ──►  Hailo Dataflow Compiler  ──►  model.hef
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import torch
import torch.nn as nn

from hailo_sdk_client import ClientRunner, InferenceContext

# ---------------------------------------------------------------------------
# Model definition (must match train_classifier.py)
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Step 1: Export to ONNX
# ---------------------------------------------------------------------------

def export_onnx(model_path: Path, out_path: Path, image_size: int) -> Path:
    checkpoint = torch.load(model_path, map_location="cpu")
    class_to_idx: dict[str, int] = checkpoint["class_to_idx"]
    num_classes = len(class_to_idx)

    model = SmallCNN(num_classes=num_classes)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    dummy = torch.zeros(1, 3, image_size, image_size)
    onnx_path = out_path / "model.onnx"

    torch.onnx.export(
        model,
        dummy,
        str(onnx_path),
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={"input": {0: "batch"}, "output": {0: "batch"}},
        opset_version=11,
    )
    print(f"ONNX model saved: {onnx_path}")
    return onnx_path


# ---------------------------------------------------------------------------
# Step 2: Hailo Dataflow Compiler — parse + optimize + quantize + compile
# ---------------------------------------------------------------------------

def compile_hailo(
    onnx_path: Path,
    calib_images: list,   # list of numpy arrays (N, H, W, C) uint8
    out_path: Path,
    hw_arch: str,
    image_size: int,
) -> Path:


    import numpy as np

    runner = ClientRunner(hw_arch=hw_arch)

    # --- Parse ONNX ---
    hn, npz_path = runner.translate_onnx_model(
        str(onnx_path),
        "small_cnn",
        start_node_names=["input"],
        end_node_names=["output"],
    )

    # --- Optimize (set input normalization: pixels already in [0,1] after /255) ---
    runner.load_model_script(
        f"normalization1 = normalization([0.0, 0.0, 0.0], [1.0, 1.0, 1.0])\n"
        f"change_output_activation(small_cnn/classifier/linear1, softmax)\n"
    )

    # --- Calibration / quantization ---
    # Calibration images: list of (H, W, 3) uint8 numpy arrays
    calib_dataset = np.stack(calib_images).astype(np.float32) / 255.0  # (N, H, W, C)
    runner.optimize(calib_dataset)

    # --- Compile to HEF ---
    hef_path = out_path / "model.hef"
    hef = runner.compile()
    with open(hef_path, "wb") as f:
        f.write(hef)

    print(f"HEF compiled: {hef_path}")
    return hef_path


# ---------------------------------------------------------------------------
# Calibration image loader
# ---------------------------------------------------------------------------

def load_calib_images(data_dir: Path, image_size: int, max_images: int = 64) -> list:
    import numpy as np
    from PIL import Image as PILImage

    calib_dir = data_dir / "train" / "track"
    if not calib_dir.exists():
        calib_dir = data_dir / "train"

    images = []
    for p in sorted(calib_dir.glob("*.png"))[:max_images]:
        img = PILImage.open(p).convert("RGB").resize((image_size, image_size))
        images.append(np.array(img))

    if not images:
        raise FileNotFoundError(f"No calibration images found in {calib_dir}")

    print(f"Loaded {len(images)} calibration images from {calib_dir}")
    return images


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export and compile SmallCNN for Hailo-8.")
    parser.add_argument("--model", type=Path, default=Path("outputs/model.pt"),
                        help="Path to the trained model.pt checkpoint.")
    parser.add_argument("--data-dir", type=Path, default=Path("data/tracks"),
                        help="Dataset root (used for calibration images).")
    parser.add_argument("--out-dir", type=Path, default=Path("outputs"),
                        help="Output directory for ONNX and HEF files.")
    parser.add_argument("--image-size", type=int, default=64,
                        help="Square input resolution (must match training). Default: 64")
    parser.add_argument("--hw-arch", default="hailo8",
                        choices=["hailo8", "hailo8l"],
                        help="Target Hailo chip. Default: hailo8")
    parser.add_argument("--calib-images", type=int, default=64,
                        help="Number of calibration images for quantization. Default: 64")
    parser.add_argument("--onnx-only", action="store_true",
                        help="Export to ONNX only; skip Hailo compilation.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    # Step 1: ONNX export
    onnx_path = export_onnx(args.model, args.out_dir, args.image_size)

    if args.onnx_only:
        print("--onnx-only set, stopping after ONNX export.")
        return

    # Step 2: Load calibration images
    calib_images = load_calib_images(args.data_dir, args.image_size, args.calib_images)

    # Step 3: Compile to HEF
    compile_hailo(onnx_path, calib_images, args.out_dir, args.hw_arch, args.image_size)


if __name__ == "__main__":
    main()
