"""Capture training images from the Pi camera.

Usage:
    python capture_data.py --output /path/to/dataset/images
    
Controls:
    SPACE - capture current frame
    q     - quit

Saves frames as numbered JPEGs into the output directory.
"""
import argparse
import os

import cv2
from picamera2 import Picamera2


def main():
    parser = argparse.ArgumentParser(description="Capture training images from Pi camera")
    parser.add_argument("--output", type=str, default="dataset/images", help="Output directory")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=640)
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    existing = [f for f in os.listdir(args.output) if f.endswith(".jpg")]
    counter = len(existing)

    camera = Picamera2()
    config = camera.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"})
    camera.configure(config)
    camera.start()

    print(f"Saving to: {args.output}")
    print("SPACE = capture, q = quit")

    try:
        while True:
            frame = camera.capture_array()
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.putText(display, f"Captured: {counter}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Capture", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                path = os.path.join(args.output, f"img_{counter:05d}.jpg")
                cv2.imwrite(path, display)
                print(f"Saved: {path}")
                counter += 1
            elif key == ord('q'):
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print(f"Total captured: {counter}")


if __name__ == "__main__":
    main()
