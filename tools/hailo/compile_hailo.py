#!/usr/bin/env python3
"""Compile an ONNX model to a Hailo HEF file."""

from __future__ import annotations

import argparse
from pathlib import Path

DEFAULT_ONNX = "outputs/model.onnx"
DEFAULT_DATA_DIR = "data/tracks"
DEFAULT_OUT_DIR = "outputs"

# Hailo Dataflow Compiler — parse + optimize + quantize + compile

def compile_hailo(
    onnx_path: Path,
    calib_images: list,   # list of numpy arrays (N, H, W, C) uint8
    out_path: Path,
    hw_arch: str,
) -> Path:
    try:
        from hailo_sdk_client import ClientRunner
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "hailo_sdk_client is required for HEF compilation. Use the Hailo SDK venv."
        ) from exc

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compile ONNX model to Hailo HEF.")
    parser.add_argument("--onnx", type=Path, default=DEFAULT_ONNX,
                        help="Path to ONNX model.")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR,
                        help="Dataset root (used for calibration images).")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR,
                        help="Output directory for HEF file.")
    parser.add_argument("--image-size", type=int, default=64,
                        help="Square input resolution (must match ONNX input). Default: 64")
    parser.add_argument("--hw-arch", default="hailo8",
                        choices=["hailo8", "hailo8l"],
                        help="Target Hailo chip. Default: hailo8")
    parser.add_argument("--calib-images", type=int, default=64,
                        help="Number of calibration images for quantization. Default: 64")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    onnx_path = args.onnx
    if not onnx_path.exists():
        raise FileNotFoundError(f"ONNX file not found: {onnx_path}")
    print(f"Using ONNX model: {onnx_path}")

    # Step 1: Load calibration images
    calib_images = load_calib_images(args.data_dir, args.image_size, args.calib_images)

    # Step 2: Compile to HEF
    compile_hailo(onnx_path, calib_images, args.out_dir, args.hw_arch)


if __name__ == "__main__":
    main()
