#!/usr/bin/env python3
"""Train a small CNN on image folders (synthetic or real webcam captures)."""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch import nn
from torch.utils.data import DataLoader, Dataset


IMAGE_EXTS = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.webp")
ANGLE_LABELS_FILE = "track_labels.csv"


class FolderDataset(Dataset):
    def __init__(self, samples: list[tuple[Path, int, float, bool]], image_size: int, augment: bool = False):
        self.samples = samples
        self.image_size = image_size
        self.augment = augment

        if not self.samples:
            raise ValueError("No images found in dataset")

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, int, float, bool]:
        image_path, label, angle_degrees, has_angle = self.samples[index]
        image = Image.open(image_path).convert("RGB")
        if self.image_size > 0 and image.size != (self.image_size, self.image_size):
            image = image.resize((self.image_size, self.image_size), Image.BILINEAR)
        tensor = torch.from_numpy(np.array(image, dtype=np.uint8))  # (H, W, 3)
        tensor = tensor.permute(2, 0, 1).float() / 255.0            # (3, H, W)
        if self.augment:
            tensor = _augment(tensor)
        return tensor, label, angle_degrees, has_angle


def _augment(tensor: torch.Tensor) -> torch.Tensor:
    """Apply mild random augmentation to bridge the synthetic-to-real domain gap."""
    # Random horizontal flip
    if torch.rand(1).item() < 0.5:
        tensor = tensor.flip(2)
    # Random vertical flip
    if torch.rand(1).item() < 0.5:
        tensor = tensor.flip(1)
    # Mild brightness shift
    if torch.rand(1).item() < 0.3:
        delta = (torch.rand(1).item() - 0.5) * 0.15  # [-0.075, +0.075]
        tensor = (tensor + delta).clamp(0.0, 1.0)
    # Mild Gaussian noise
    if torch.rand(1).item() < 0.2:
        noise = torch.randn_like(tensor) * 0.03
        tensor = (tensor + noise).clamp(0.0, 1.0)
    return tensor


def load_angle_targets(root: Path) -> dict[str, float]:
    labels_path = root / ANGLE_LABELS_FILE
    if not labels_path.is_file():
        return {}

    angle_targets: dict[str, float] = {}
    with labels_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            image_rel = row.get("image")
            angle_text = row.get("angle_degrees")
            if not image_rel or angle_text is None:
                continue
            angle_targets[image_rel] = float(angle_text)

    return angle_targets


def collect_samples(root: Path) -> tuple[list[tuple[Path, int, float, bool]], dict[str, int]]:
    classes = sorted([p.name for p in root.iterdir() if p.is_dir()])
    if not classes:
        raise ValueError(f"No class folders found in {root}")

    class_to_idx = {name: idx for idx, name in enumerate(classes)}
    angle_targets = load_angle_targets(root)
    samples: list[tuple[Path, int, float, bool]] = []

    for class_name, class_idx in class_to_idx.items():
        class_dir = root / class_name
        class_images: list[Path] = []
        for pattern in IMAGE_EXTS:
            class_images.extend(class_dir.glob(pattern))
        for image_path in sorted(class_images):
            rel_path = image_path.relative_to(root).as_posix()
            angle_degrees = float(angle_targets.get(rel_path, 0.0))
            has_angle = rel_path in angle_targets
            samples.append((image_path, class_idx, angle_degrees, has_angle))

    if not samples:
        raise ValueError(f"No images found in class folders under {root}")

    return samples, class_to_idx


class SmallCNN(nn.Module):
    def __init__(self, num_classes: int, predict_angle: bool = False):
        super().__init__()
        self.predict_angle = predict_angle
        self.features = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.drop = nn.Dropout(0.3)
        self.classifier = nn.Linear(128, num_classes)
        self.angle_head = nn.Linear(128, 2) if predict_angle else None

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor | None]:
        x = self.features(x)
        x = x.view(x.size(0), -1)
        x = self.drop(x)
        logits = self.classifier(x)
        angle_vector = self.angle_head(x) if self.angle_head is not None else None
        return logits, angle_vector


def angle_degrees_to_vector(angle_degrees: torch.Tensor) -> torch.Tensor:
    angle_radians = torch.deg2rad(angle_degrees)
    return torch.stack((torch.sin(angle_radians), torch.cos(angle_radians)), dim=1)


def angle_vector_to_degrees(angle_vector: torch.Tensor) -> torch.Tensor:
    angle_radians = torch.atan2(angle_vector[:, 0], angle_vector[:, 1])
    return torch.rad2deg(angle_radians).remainder(360.0)


def angular_error_degrees(predicted: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    return ((predicted - target + 180.0).remainder(360.0) - 180.0).abs()


def evaluate(
    model: nn.Module,
    loader: DataLoader,
    device: torch.device,
    predict_angle: bool,
) -> tuple[float, float, float | None]:
    model.eval()
    criterion = nn.CrossEntropyLoss()
    total_loss = 0.0
    correct = 0
    total = 0
    angle_error_sum = 0.0
    angle_count = 0

    with torch.no_grad():
        for inputs, labels, angle_degrees, has_angle in loader:
            inputs = inputs.to(device)
            labels = labels.to(device)
            angle_degrees = angle_degrees.to(device=device, dtype=torch.float32)
            has_angle = has_angle.to(device=device, dtype=torch.bool)

            logits, angle_vector = model(inputs)
            loss = criterion(logits, labels)

            total_loss += loss.item() * inputs.size(0)
            preds = logits.argmax(dim=1)
            correct += (preds == labels).sum().item()
            total += inputs.size(0)

            if predict_angle and angle_vector is not None and has_angle.any():
                predicted_angle = angle_vector_to_degrees(angle_vector[has_angle])
                angle_error = angular_error_degrees(predicted_angle, angle_degrees[has_angle])
                angle_error_sum += angle_error.sum().item()
                angle_count += int(has_angle.sum().item())

    mean_angle_error = angle_error_sum / angle_count if angle_count > 0 else None
    return total_loss / total, correct / total, mean_angle_error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--cpu", action="store_true", help="Force CPU even if CUDA is available")
    parser.add_argument("--image-size", type=int, default=128, help="Resize input images to a square size")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    t0 = time.monotonic()

    train_root = args.data_dir / "train"
    val_root = args.data_dir / "val"

    if not train_root.is_dir() or not val_root.is_dir():
        raise ValueError(
            f"Expected explicit dataset split with class folders under {train_root} and {val_root}."
        )

    train_samples, class_to_idx = collect_samples(train_root)
    val_samples, val_class_to_idx = collect_samples(val_root)
    if class_to_idx != val_class_to_idx:
        raise ValueError("Train and validation class folders do not match")
    print(f"Dataset mode: explicit split ({train_root} / {val_root})")

    predict_angle = any(has_angle for _, _, _, has_angle in train_samples)

    train_ds = FolderDataset(train_samples, image_size=args.image_size, augment=True)
    val_ds = FolderDataset(val_samples, image_size=args.image_size, augment=False)

    use_cuda = torch.cuda.is_available() and not args.cpu

    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=use_cuda,
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=use_cuda,
    )

    device = torch.device("cuda" if use_cuda else "cpu")
    model = SmallCNN(num_classes=len(class_to_idx), predict_angle=predict_angle).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.CrossEntropyLoss()
    angle_criterion = nn.MSELoss()
    angle_loss_weight = 0.25

    print(f"Using device: {device}")
    print(f"Input size: {args.image_size}x{args.image_size}")
    print(f"Classes: {class_to_idx}")
    print(f"Train samples: {len(train_ds)}, Val samples: {len(val_ds)}")
    print(f"Angle head: {'enabled' if predict_angle else 'disabled'}")

    best_val_acc = 0.0
    best_state: dict | None = None
    best_epoch = 0

    for epoch in range(1, args.epochs + 1):
        model.train()
        running_loss = 0.0
        correct = 0
        total = 0
        angle_error_sum = 0.0
        angle_count = 0

        for inputs, labels, angle_degrees, has_angle in train_loader:
            inputs = inputs.to(device)
            labels = labels.to(device)
            angle_degrees = angle_degrees.to(device=device, dtype=torch.float32)
            has_angle = has_angle.to(device=device, dtype=torch.bool)

            optimizer.zero_grad()
            logits, angle_vector = model(inputs)
            loss = criterion(logits, labels)

            if predict_angle and angle_vector is not None and has_angle.any():
                target_angle_vector = angle_degrees_to_vector(angle_degrees[has_angle])
                angle_loss = angle_criterion(angle_vector[has_angle], target_angle_vector)
                loss = loss + angle_loss_weight * angle_loss

                predicted_angle = angle_vector_to_degrees(angle_vector[has_angle].detach())
                angle_error = angular_error_degrees(predicted_angle, angle_degrees[has_angle])
                angle_error_sum += angle_error.sum().item()
                angle_count += int(has_angle.sum().item())

            loss.backward()
            optimizer.step()

            running_loss += loss.item() * inputs.size(0)
            preds = logits.argmax(dim=1)
            correct += (preds == labels).sum().item()
            total += inputs.size(0)

        train_loss = running_loss / total
        train_acc = correct / total
        train_angle_error = angle_error_sum / angle_count if angle_count > 0 else None
        val_loss, val_acc, val_angle_error = evaluate(model, val_loader, device, predict_angle)

        # Save best model by val accuracy
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}
            best_epoch = epoch

        epoch_message = (
            f"Epoch {epoch:02d}/{args.epochs} | "
            f"train loss {train_loss:.4f} acc {train_acc:.3f}"
        )
        if train_angle_error is not None:
            epoch_message += f" angle_mae {train_angle_error:.1f}deg"
        epoch_message += f" | val loss {val_loss:.4f} acc {val_acc:.3f}"
        if val_angle_error is not None:
            epoch_message += f" angle_mae {val_angle_error:.1f}deg"
        print(epoch_message)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.out_dir / "model.pt"
    save_state = best_state if best_state is not None else model.state_dict()
    torch.save(
        {
            "model_state_dict": save_state,
            "class_to_idx": class_to_idx,
            "image_size": args.image_size,
            "predict_angle": predict_angle,
        },
        out_path,
    )

    meta_path = args.out_dir / "classes.json"
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(class_to_idx, f, indent=2)

    elapsed = time.monotonic() - t0
    print(f"Saved best model (epoch {best_epoch}, val acc {best_val_acc:.3f}) to: {out_path}")
    print(f"Saved classes to: {meta_path}")
    print(f"Total time: {elapsed:.1f}s")


if __name__ == "__main__":
    main()
