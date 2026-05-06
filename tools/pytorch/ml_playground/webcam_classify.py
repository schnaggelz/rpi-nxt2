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
from train_classifier import angle_vector_to_degrees


def draw_probability_panel(
    frame: np.ndarray,
    probs: torch.Tensor,
    idx_to_class: dict[int, str],
    pred_class: str,
    confidence: float,
    margin: float,
    entropy: float,
    input_size: int,
    device_name: str,
    angle_degrees: float | None,
    angle_source: str | None,
) -> None:
    panel_width = 320
    x0 = max(0, frame.shape[1] - panel_width - 12)
    y0 = 12
    x1 = frame.shape[1] - 12
    y1 = min(frame.shape[0] - 12, 190 + 28 * len(idx_to_class))

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x1, y1), (20, 20, 20), thickness=-1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

    lines = [
        f"pred: {pred_class}",
        f"confidence: {confidence:.1%}",
        f"margin: {margin:.1%}",
        f"entropy: {entropy:.3f}",
        f"input: {input_size}x{input_size}",
        f"device: {device_name}",
    ]
    if angle_degrees is not None and angle_source is not None:
        lines.append(f"{angle_source}: {angle_degrees:.1f} deg")

    y = y0 + 24
    for line in lines:
        cv2.putText(frame, line, (x0 + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        y += 22

    y += 8
    bar_left = x0 + 90
    bar_right = x1 - 16
    bar_width = max(40, bar_right - bar_left)
    for idx in sorted(idx_to_class):
        class_name = idx_to_class[idx]
        prob = probs[idx].item()
        cv2.putText(frame, class_name, (x0 + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.rectangle(frame, (bar_left, y - 10), (bar_right, y), (80, 80, 80), thickness=1)
        fill_right = bar_left + int(prob * bar_width)
        cv2.rectangle(frame, (bar_left, y - 10), (fill_right, y), (0, 180, 255), thickness=-1)
        cv2.putText(frame, f"{prob:.1%}", (bar_left, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        y += 28


def draw_input_preview(frame: np.ndarray, model_input_bgr: np.ndarray) -> None:
    preview_size = 96
    preview = cv2.resize(model_input_bgr, (preview_size, preview_size), interpolation=cv2.INTER_NEAREST)
    x0 = 12
    y0 = max(12, frame.shape[0] - preview_size - 36)
    x1 = x0 + preview_size
    y1 = y0 + preview_size
    frame[y0:y1, x0:x1] = preview
    cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 255, 255), 1)
    cv2.putText(frame, "model input", (x0, y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


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
    parser.add_argument("--no-display", action="store_true", help="Run without GUI window (headless mode)")
    parser.add_argument("--print-every", type=int, default=30, help="In no-display mode, print every N frames")
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
    checkpoint_input_size = int(checkpoint.get("image_size", args.size))
    input_size = args.size or checkpoint_input_size
    predict_angle = bool(checkpoint.get("predict_angle", False))

    device = torch.device("cpu" if args.cpu or not torch.cuda.is_available() else "cuda")
    model = SmallCNN(num_classes=len(class_to_idx), predict_angle=predict_angle).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    print(f"Loaded model from {args.model} ({len(class_to_idx)} classes: {list(class_to_idx.keys())})")
    print(f"Using device: {device}")
    print(f"Classifier outputs: class probabilities for {list(class_to_idx.keys())}")
    if predict_angle:
        print("Additional output: track-angle estimate")

    show_window = not args.no_display
    if show_window and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("No display server detected; switching to --no-display mode.")
        show_window = False

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Error: cannot open camera {args.camera}")
        return

    if show_window:
        print("Press 'q' to quit.")
    else:
        print("Headless mode active. Press Ctrl+C to quit.")

    frame_count = 0
    with torch.no_grad():
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            frame_count += 1

            # Preprocess: resize, convert BGR->RGB, normalize
            resized = cv2.resize(frame, (input_size, input_size))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            tensor = torch.from_numpy(rgb.astype(np.float32)).permute(2, 0, 1) / 255.0
            tensor = tensor.unsqueeze(0).to(device)

            logits, angle_vector = model(tensor)
            probs = torch.softmax(logits, dim=1)[0]
            pred_idx = probs.argmax().item()
            pred_class = idx_to_class[pred_idx]
            confidence = probs[pred_idx].item()
            sorted_probs, _sorted_idx = torch.sort(probs, descending=True)
            second_best = sorted_probs[1].item() if len(sorted_probs) > 1 else 0.0
            margin = confidence - second_best
            entropy = float(-(probs * torch.log(probs.clamp_min(1e-8))).sum().item())
            angle_degrees: float | None = None
            if predict_angle and angle_vector is not None:
                angle_degrees = float(angle_vector_to_degrees(angle_vector).item())

            # Draw prediction on frame
            label = f"{pred_class} ({confidence:.0%})"
            cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            draw_probability_panel(
                frame,
                probs,
                idx_to_class,
                pred_class,
                confidence,
                margin,
                entropy,
                input_size,
                str(device),
                angle_degrees,
                "track angle",
            )
            draw_input_preview(frame, resized)

            if show_window:
                cv2.imshow("Classifier", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            elif args.print_every > 0 and frame_count % args.print_every == 0:
                prob_summary = ", ".join(
                    f"{idx_to_class[idx]}={probs[idx].item():.1%}" for idx in sorted(idx_to_class)
                )
                print(
                    f"frame {frame_count}: pred={pred_class} conf={confidence:.1%} "
                    f"margin={margin:.1%} entropy={entropy:.3f}"
                    + (f" track_angle={angle_degrees:.1f}deg" if angle_degrees is not None else "")
                    + f" | {prob_summary}"
                )

    cap.release()
    if show_window:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
