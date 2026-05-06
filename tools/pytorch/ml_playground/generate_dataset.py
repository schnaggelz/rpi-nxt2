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


_SIGN_COLORS: list[tuple[int, int, int]] = [
    (235, 110, 40),
    (245, 120, 50),
    (225, 95, 35),
]


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


def make_sign_image(
    image_size: int,
) -> tuple[Image.Image, dict[str, float | int | str]]:
    # White-ish plate background.
    bg_level = random.randint(215, 250)
    img = Image.new("RGB", (image_size, image_size), color=(bg_level, bg_level, bg_level))
    draw = ImageDraw.Draw(img)
    _add_background_noise(draw, image_size)

    # Draw the orange sign marker – large to match close-up camera view.
    radius = random.uniform(image_size * 0.15, image_size * 0.30)
    margin = radius + 4
    sx = random.uniform(margin, image_size - margin)
    sy = random.uniform(margin, image_size - margin)
    sign_color = random.choice(_SIGN_COLORS)
    draw.ellipse([sx - radius, sy - radius, sx + radius, sy + radius], fill=sign_color)

    if random.random() < 0.15:
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.1, 0.5)))

    return img, {
        "sign_radius_px": round(radius, 2),
        "sign_rgb": f"{sign_color[0]}-{sign_color[1]}-{sign_color[2]}",
    }


def _track_color_and_thickness(
    image_size: int,
) -> tuple[int, tuple[int, int, int], str]:
    thickness = random.randint(int(image_size * 0.35), int(image_size * 0.65))
    if random.random() < 0.85:
        tone = random.randint(0, 30)
        color: tuple[int, int, int] = (tone, tone, tone)
        name = "black"
    else:
        warm = random.randint(0, 40)
        color = (warm + 10, warm, warm)
        name = "dark"
    return thickness, color, name


def make_straight_image(
    image_size: int,
) -> tuple[Image.Image, dict[str, float | int | str]]:
    bg_level = random.randint(170, 245)
    bg = (bg_level, bg_level, bg_level)
    img = Image.new("RGB", (image_size, image_size), color=bg)
    draw = ImageDraw.Draw(img)
    _add_background_noise(draw, image_size)

    thickness, track_color, color_name = _track_color_and_thickness(image_size)
    half = image_size / 2

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

    if random.random() < 0.2:
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.15, 0.6)))

    metadata: dict[str, float | int | str] = {
        "angle_degrees": round(angle_deg, 3),
        "thickness_px": thickness,
        "track_color": color_name,
    }
    return img, metadata


def make_curve_image(
    image_size: int,
) -> tuple[Image.Image, dict[str, float | int | str]]:
    bg_level = random.randint(170, 245)
    bg = (bg_level, bg_level, bg_level)
    img = Image.new("RGB", (image_size, image_size), color=bg)
    draw = ImageDraw.Draw(img)
    _add_background_noise(draw, image_size)

    thickness, track_color, color_name = _track_color_and_thickness(image_size)
    half = image_size / 2

    # Circular arc whose center-tangent is the labelled angle.
    # Tight radius to match real tape curves visible in close-up camera view.
    radius = random.uniform(image_size * 0.4, image_size * 1.5)
    dir_angle = random.uniform(0.0, 2 * math.pi)
    dist = radius + random.uniform(-image_size * 0.1, image_size * 0.1)
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

    if random.random() < 0.2:
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.15, 0.6)))

    metadata: dict[str, float | int | str] = {
        "angle_degrees": round(angle_deg, 3),
        "thickness_px": thickness,
        "curvature_radius_px": curvature_radius,
        "track_color": color_name,
    }
    return img, metadata


def make_crossing_image(
    image_size: int,
) -> tuple[Image.Image, dict[str, float | int | str]]:
    bg_level = random.randint(170, 245)
    bg = (bg_level, bg_level, bg_level)
    img = Image.new("RGB", (image_size, image_size), color=bg)
    draw = ImageDraw.Draw(img)
    _add_background_noise(draw, image_size)

    # Thick tape: 30-55% of image size to match real close-up camera view
    thickness = random.randint(int(image_size * 0.30), int(image_size * 0.55))
    if random.random() < 0.85:
        tone = random.randint(0, 30)
        track_color: tuple[int, int, int] = (tone, tone, tone)
        color_name = "black"
    else:
        warm = random.randint(0, 40)
        track_color = (warm + 10, warm, warm)
        color_name = "dark"

    half = image_size / 2

    # Crossing: two lines perpendicular to each other, intersecting at center
    # This makes crossings clearly distinct from single-line tracks
    base_angle = random.uniform(0.0, 90.0)  # Only need 0-90° since perpendicular covers full circle
    angle1_deg = base_angle
    angle2_deg = (base_angle + 90.0) % 360

    # Intersection point at center (not random) to make crossing pattern clear
    cx = half + random.uniform(-half * 0.1, half * 0.1)  # Slight wobble, not-too-much
    cy = half + random.uniform(-half * 0.1, half * 0.1)
    reach = image_size * 0.95

    for angle_deg in (angle1_deg, angle2_deg):
        angle_rad = math.radians(angle_deg)
        dx, dy = math.cos(angle_rad), math.sin(angle_rad)
        start = (cx - dx * reach, cy - dy * reach)
        end = (cx + dx * reach, cy + dy * reach)
        draw.line([start, end], fill=track_color, width=thickness)

    if random.random() < 0.2:
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.15, 0.6)))

    # Always perpendicular (90°)
    intersection_angle = 90.0

    metadata: dict[str, float | int | str] = {
        "angle1_degrees": round(angle1_deg, 3),
        "angle2_degrees": round(angle2_deg, 3),
        "intersection_angle": round(intersection_angle, 3),
        "thickness_px": thickness,
        "track_color": color_name,
    }
    return img, metadata


def write_split(
    root: Path,
    split: str,
    image_count: int,
    image_size: int,
) -> None:
    straight_dir = root / split / "straight"
    straight_dir.mkdir(parents=True, exist_ok=True)
    straight_meta = root / split / "straight_labels.csv"

    with straight_meta.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=["image", "class_name", "angle_degrees", "thickness_px", "track_color"],
        )
        writer.writeheader()
        for idx in range(image_count):
            image_name = f"{idx:04d}.png"
            img, metadata = make_straight_image(image_size)
            img.save(straight_dir / image_name)
            writer.writerow({"image": f"straight/{image_name}", "class_name": "straight", **metadata})

    curve_dir = root / split / "curve"
    curve_dir.mkdir(parents=True, exist_ok=True)
    curve_meta = root / split / "curve_labels.csv"

    with curve_meta.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=["image", "class_name", "angle_degrees", "thickness_px", "curvature_radius_px", "track_color"],
        )
        writer.writeheader()
        for idx in range(image_count):
            image_name = f"{idx:04d}.png"
            img, metadata = make_curve_image(image_size)
            img.save(curve_dir / image_name)
            writer.writerow({"image": f"curve/{image_name}", "class_name": "curve", **metadata})

    crossing_dir = root / split / "crossing"
    crossing_dir.mkdir(parents=True, exist_ok=True)
    crossing_meta = root / split / "crossing_labels.csv"
    with crossing_meta.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=["image", "class_name", "angle1_degrees", "angle2_degrees",
                         "intersection_angle", "thickness_px", "track_color"],
        )
        writer.writeheader()
        for idx in range(image_count):
            image_name = f"{idx:04d}.png"
            img, metadata = make_crossing_image(image_size)
            img.save(crossing_dir / image_name)
            writer.writerow({"image": f"crossing/{image_name}", "class_name": "crossing", **metadata})

    sign_dir = root / split / "sign"
    sign_dir.mkdir(parents=True, exist_ok=True)
    sign_meta = root / split / "sign_labels.csv"
    with sign_meta.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=["image", "class_name", "sign_radius_px", "sign_rgb"],
        )
        writer.writeheader()
        for idx in range(image_count):
            image_name = f"{idx:04d}.png"
            img, metadata = make_sign_image(image_size)
            img.save(sign_dir / image_name)
            writer.writerow({"image": f"sign/{image_name}", "class_name": "sign", **metadata})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a synthetic track/line dataset.")
    parser.add_argument("--out-dir", type=Path, default=Path("data"))
    parser.add_argument("--image-size", type=int, default=64)
    parser.add_argument("--train-count", type=int, default=400)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove only --out-dir/train before generation (val is preserved)",
    )
    args = parser.parse_args()

    return args


def main() -> None:
    args = parse_args()
    random.seed(args.seed)

    train_root = args.out_dir / "train"
    if args.clean and train_root.exists():
        shutil.rmtree(train_root)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    write_split(
        args.out_dir, "train", args.train_count,
        args.image_size,
    )

    print(f"Dataset created in: {args.out_dir}")
    print("Splits generated: train")
    print(f"Train images per class: {args.train_count}")
    print("Classes: straight, curve, crossing, sign")
    print("Metadata: train/straight_labels.csv, train/curve_labels.csv,")
    print("          train/crossing_labels.csv, train/sign_labels.csv")


if __name__ == "__main__":
    main()
