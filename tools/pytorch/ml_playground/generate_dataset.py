#!/usr/bin/env python3
"""Generate a synthetic track dataset: thick, mostly black lines at arbitrary angles."""

from __future__ import annotations

import argparse
import csv
import math
import random
import shutil
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter


def _add_background_noise(draw: ImageDraw.ImageDraw, image_size: int) -> None:
    for _ in range(random.randint(15, 40)):
        x = random.randint(0, image_size - 1)
        y = random.randint(0, image_size - 1)
        n = random.randint(70, 120)
        draw.point((x, y), fill=(n, n, n))


_TRAFFIC_COLORS: dict[str, tuple[int, int, int]] = {
    "go":   (30, 180, 50),
    "stop": (200, 30, 30),
}


def _arc_polyline(
    cx: float,
    cy: float,
    radius: float,
    start_angle_rad: float,
    sweep_rad: float,
) -> list[tuple[float, float]]:
    # ~1 point per pixel of arc length so stamp circles never leave a gap
    n_seg = max(32, int(abs(sweep_rad) * radius))
    return [
        (cx + radius * math.cos(start_angle_rad + i / n_seg * sweep_rad),
         cy + radius * math.sin(start_angle_rad + i / n_seg * sweep_rad))
        for i in range(n_seg + 1)
    ]


def make_traffic_light_image(
    image_size: int,
) -> tuple[Image.Image, dict[str, float | int | str]]:
    bg_level = random.randint(170, 245)
    img = Image.new("RGB", (image_size, image_size), color=(bg_level, bg_level, bg_level))
    draw = ImageDraw.Draw(img)
    _add_background_noise(draw, image_size)

    signal = random.choice(["go", "stop"])
    count = random.choices([1, 2], weights=[0.75, 0.25])[0]
    box_size = max(4, image_size // 8)
    margin = 4
    used: list[tuple[int, int]] = []
    color = _TRAFFIC_COLORS[signal]

    for _ in range(count):
        for _attempt in range(30):
            x = random.randint(margin, image_size - box_size - margin)
            y = random.randint(margin, image_size - box_size - margin)
            if any(abs(x - px) < box_size * 2 and abs(y - py) < box_size * 2 for px, py in used):
                continue
            used.append((x, y))
            draw.rectangle([x, y, x + box_size, y + box_size], fill=color, outline=(0, 0, 0), width=1)
            break

    if random.random() < 0.15:
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.1, 0.5)))

    return img, {
        "traffic_lights": len(used),
        "signal": signal,
    }


def make_track_image(
    image_size: int,
    track_width_min: int,
    track_width_max: int,
    black_track_prob: float,
) -> tuple[Image.Image, dict[str, float | int | str]]:
    bg_level = random.randint(170, 245)
    bg = (bg_level, bg_level, bg_level)
    img = Image.new("RGB", (image_size, image_size), color=bg)
    draw = ImageDraw.Draw(img)
    _add_background_noise(draw, image_size)

    thickness = random.randint(track_width_min, track_width_max)
    if random.random() < black_track_prob:
        tone = random.randint(0, 30)
        track_color: tuple[int, int, int] = (tone, tone, tone)
        color_name = "black"
    else:
        warm = random.randint(0, 40)
        track_color = (warm + 10, warm, warm)
        color_name = "dark"

    half = image_size / 2
    use_curve = random.random() < 0.5

    if use_curve:
        # Circular arc whose center-tangent is the labelled angle.
        radius = random.uniform(image_size * 0.7, image_size * 3.0)
        dir_angle = random.uniform(0.0, 2 * math.pi)
        dist = radius + random.uniform(-image_size * 0.2, image_size * 0.2)
        ccx = half + math.cos(dir_angle) * dist
        ccy = half + math.sin(dir_angle) * dist
        to_img = math.atan2(half - ccy, half - ccx)
        sweep = random.uniform(math.pi / 3, math.pi) * random.choice([-1, 1])
        start_angle = to_img - sweep / 2
        points = _arc_polyline(ccx, ccy, radius, start_angle, sweep)
        r = thickness / 2
        for px, py in points:
            draw.ellipse([px - r, py - r, px + r, py + r], fill=track_color)
        sign = 1 if sweep > 0 else -1
        angle_deg = math.degrees(to_img + sign * math.pi / 2) % 360
        curvature_radius: float = round(radius, 1)
    else:
        angle_deg = random.uniform(0.0, 360.0)
        angle_rad = math.radians(angle_deg)
        direction = (math.cos(angle_rad), math.sin(angle_rad))
        normal = (-direction[1], direction[0])
        offset = random.uniform(-half * 0.5, half * 0.5)
        cx = half + normal[0] * offset
        cy = half + normal[1] * offset
        reach = image_size * 0.9
        start = (cx - direction[0] * reach, cy - direction[1] * reach)
        end   = (cx + direction[0] * reach, cy + direction[1] * reach)
        draw.line([start, end], fill=track_color, width=thickness)
        curvature_radius = -1.0  # straight

    if random.random() < 0.2:
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.15, 0.6)))

    metadata: dict[str, float | int | str] = {
        "angle_degrees": round(angle_deg, 3),
        "thickness_px": thickness,
        "curvature_radius_px": curvature_radius,
        "track_color": color_name,
    }
    return img, metadata


def write_split(
    root: Path,
    split: str,
    image_count: int,
    image_size: int,
    track_width_min: int,
    track_width_max: int,
    black_track_prob: float,
) -> None:
    track_dir = root / split / "track"
    track_dir.mkdir(parents=True, exist_ok=True)
    track_meta = root / split / "track_labels.csv"

    with track_meta.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=["image", "class_name", "angle_degrees", "thickness_px", "curvature_radius_px", "track_color"],
        )
        writer.writeheader()
        for idx in range(image_count):
            image_name = f"{idx:04d}.png"
            img, metadata = make_track_image(image_size, track_width_min, track_width_max, black_track_prob)
            img.save(track_dir / image_name)
            writer.writerow({"image": f"track/{image_name}", "class_name": "track", **metadata})

    tl_dir = root / split / "traffic_light"
    tl_dir.mkdir(parents=True, exist_ok=True)
    tl_meta = root / split / "traffic_light_labels.csv"
    with tl_meta.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=["image", "class_name", "traffic_lights", "signal"],
        )
        writer.writeheader()
        for idx in range(image_count):
            image_name = f"{idx:04d}.png"
            img, metadata = make_traffic_light_image(image_size)
            img.save(tl_dir / image_name)
            writer.writerow({"image": f"traffic_light/{image_name}", "class_name": "traffic_light", **metadata})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a synthetic track/line dataset.")
    parser.add_argument("--out-dir", type=Path, default=Path("data"))
    parser.add_argument("--image-size", type=int, default=64)
    parser.add_argument("--train-count", type=int, default=400)
    parser.add_argument("--val-count", type=int, default=100)
    parser.add_argument("--track-width-min", type=int, default=8)
    parser.add_argument("--track-width-max", type=int, default=18)
    parser.add_argument(
        "--black-track-prob",
        type=float,
        default=0.85,
        help="Probability a track is pure black (vs. a dark tone). Default: 0.85",
    )
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    if args.track_width_min < 1:
        parser.error("--track-width-min must be at least 1")
    if args.track_width_max < args.track_width_min:
        parser.error("--track-width-max must be >= --track-width-min")
    if not 0.0 <= args.black_track_prob <= 1.0:
        parser.error("--black-track-prob must be between 0.0 and 1.0")

    return args


def main() -> None:
    args = parse_args()
    random.seed(args.seed)

    if args.out_dir.exists():
        shutil.rmtree(args.out_dir)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    write_split(
        args.out_dir, "train", args.train_count,
        args.image_size, args.track_width_min, args.track_width_max, args.black_track_prob,
    )
    write_split(
        args.out_dir, "val", args.val_count,
        args.image_size, args.track_width_min, args.track_width_max, args.black_track_prob,
    )

    print(f"Dataset created in: {args.out_dir}")
    print(f"Train images per class: {args.train_count}, Val images per class: {args.val_count}")
    print("Classes: track, traffic_light (signal: go/stop)")
    print("Metadata: train/track_labels.csv, train/traffic_light_labels.csv (and val/*)") 


if __name__ == "__main__":
    main()
