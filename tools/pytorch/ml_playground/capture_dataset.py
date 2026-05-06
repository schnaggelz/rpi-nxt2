#!/usr/bin/env python3
"""Capture webcam training images manually from the webcam.

Controls:
- Space: capture one photo
- q: quit
"""

from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, default=Path("data"))
    parser.add_argument("--label", type=str, default="unlabeled", help="Class label / subfolder name")
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument("--prefix", type=str, default="img", help="Filename prefix")
    return parser.parse_args()


def next_index(folder: Path, prefix: str) -> int:
    """Find next numeric index based on existing files with the same prefix."""
    max_idx = -1
    for path in folder.glob(f"{prefix}_*.jpg"):
        stem = path.stem
        # Expected pattern: prefix_YYYYmmdd_HHMMSS_index
        parts = stem.split("_")
        if len(parts) < 4:
            continue
        try:
            idx = int(parts[-1])
        except ValueError:
            continue
        if idx > max_idx:
            max_idx = idx
    return max_idx + 1


def main() -> None:
    args = parse_args()

    class_dir = args.out_dir / args.label
    class_dir.mkdir(parents=True, exist_ok=True)

    manifest_path = args.out_dir / "captures.csv"
    is_new_manifest = not manifest_path.exists()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Error: could not open camera {args.camera}")
        return

    frame_idx = next_index(class_dir, args.prefix)
    saved_count = 0

    with manifest_path.open("a", newline="", encoding="utf-8") as manifest_file:
        writer = csv.writer(manifest_file)
        if is_new_manifest:
            writer.writerow(["file", "label", "timestamp_iso", "epoch_ms"])

        print("Preview opened.")
        print("SPACE: capture one photo | q: quit")

        while True:
            ok, frame = cap.read()
            if not ok:
                print("Warning: failed to read frame from camera")
                break

            # Keep a clean frame for saving; preview overlays are added below.
            raw_frame = frame.copy()

            status = "READY"
            status_color = (0, 200, 0)
            cv2.putText(frame, f"Status: {status}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            cv2.putText(
                frame,
                f"Saved: {saved_count} | Label: {args.label}",
                (12, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                frame,
                "SPACE capture, q quit",
                (12, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
            )

            cv2.imshow("Webcam Dataset Capture", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            if key == ord(" "):
                wall_time = datetime.now()
                filename = f"{args.prefix}_{wall_time:%Y%m%d_%H%M%S}_{frame_idx:06d}.jpg"
                image_path = class_dir / filename
                cv2.imwrite(str(image_path), raw_frame)

                writer.writerow(
                    [
                        str(image_path.relative_to(args.out_dir)),
                        args.label,
                        wall_time.isoformat(timespec="milliseconds"),
                        int(time.time() * 1000),
                    ]
                )
                manifest_file.flush()

                saved_count += 1
                frame_idx += 1
                print(f"Saved: {image_path}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"Done. Total saved this run: {saved_count}")


if __name__ == "__main__":
    main()
