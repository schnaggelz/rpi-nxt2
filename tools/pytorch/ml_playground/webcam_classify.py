#!/usr/bin/env python3
"""Live webcam classification using a trained SmallCNN model."""

from __future__ import annotations

import argparse
import glob
import os
from pathlib import Path

# Keep camera discovery output readable by default.
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np
import torch

from train_classifier import SmallCNN


def _set_opencv_log_quiet() -> None:
    """Lower OpenCV logging noise during camera probing."""
    # OpenCV log-level APIs differ by version; best-effort only.
    if hasattr(cv2, "setLogLevel") and hasattr(cv2, "LOG_LEVEL_ERROR"):
        cv2.setLogLevel(cv2.LOG_LEVEL_ERROR)


def list_cameras() -> list[dict[str, int | str]]:
    """Probe available /dev/video* devices and return camera info."""
    cameras: list[dict[str, int | str]] = []

    _set_opencv_log_quiet()

    # Probe only existing Linux V4L2 device nodes to avoid index spam.
    dev_videos = sorted(glob.glob("/dev/video*"))
    indices: set[int] = set()
    for dev in dev_videos:
        try:
            indices.add(int(Path(dev).name.replace("video", "")))
        except ValueError:
            pass

    for idx in sorted(indices):
        dev_path = f"/dev/video{idx}"
        cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue

        # Ensure the stream can actually provide frames.
        ok, _frame = cap.read()
        if not ok:
            cap.release()
            continue

        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        backend = cap.getBackendName()
        cameras.append({"index": idx, "width": w, "height": h, "fps": fps, "backend": backend})
        cap.release()

    return cameras


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=Path("outputs/model.pt"))
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--size", type=int, default=64, help="Input size for the model")
    parser.add_argument("--cpu", action="store_true", help="Force CPU even if CUDA is available")
    parser.add_argument("--list-cameras", action="store_true", help="Discover available cameras and exit")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_cameras:
        cameras = list_cameras()
        if not cameras:
            print("No cameras found.")
        else:
            print(f"Found {len(cameras)} camera(s):")
            for cam in cameras:
                print(f"  index {cam['index']}: {cam['width']}x{cam['height']} @ {cam['fps']:.0f} fps ({cam['backend']})")
        return

    checkpoint = torch.load(args.model, map_location="cpu", weights_only=False)
    class_to_idx: dict[str, int] = checkpoint["class_to_idx"]
    idx_to_class = {v: k for k, v in class_to_idx.items()}

    device = torch.device("cpu" if args.cpu or not torch.cuda.is_available() else "cuda")
    model = SmallCNN(num_classes=len(class_to_idx)).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    print(f"Loaded model from {args.model} ({len(class_to_idx)} classes: {list(class_to_idx.keys())})")
    print(f"Using device: {device}")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Error: cannot open camera {args.camera}")
        return

    print("Press 'q' to quit.")

    with torch.no_grad():
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Preprocess: resize, convert BGR->RGB, normalize
            resized = cv2.resize(frame, (args.size, args.size))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            tensor = torch.from_numpy(rgb.astype(np.float32)).permute(2, 0, 1) / 255.0
            tensor = tensor.unsqueeze(0).to(device)

            logits = model(tensor)
            probs = torch.softmax(logits, dim=1)[0]
            pred_idx = probs.argmax().item()
            pred_class = idx_to_class[pred_idx]
            confidence = probs[pred_idx].item()

            # Draw prediction on frame
            label = f"{pred_class} ({confidence:.0%})"
            cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # Draw all class probabilities
            y = 60
            for idx in sorted(idx_to_class.keys()):
                name = idx_to_class[idx]
                prob = probs[idx].item()
                text = f"{name}: {prob:.1%}"
                cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                y += 20

            cv2.imshow("Classifier", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
