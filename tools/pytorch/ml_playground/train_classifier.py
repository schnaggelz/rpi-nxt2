#!/usr/bin/env python3
"""Train a small CNN on the synthetic dataset (track / traffic_light classes)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch import nn
from torch.utils.data import DataLoader, Dataset


class ShapesDataset(Dataset):
    def __init__(self, root: Path):
        self.samples: list[tuple[Path, int]] = []
        self.class_to_idx: dict[str, int] = {}

        classes = sorted([p.name for p in root.iterdir() if p.is_dir()])
        self.class_to_idx = {name: idx for idx, name in enumerate(classes)}

        for class_name, class_idx in self.class_to_idx.items():
            class_dir = root / class_name
            for image_path in sorted(class_dir.glob("*.png")):
                self.samples.append((image_path, class_idx))

        if not self.samples:
            raise ValueError(f"No PNG images found in {root}")

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, int]:
        image_path, label = self.samples[index]
        image = Image.open(image_path).convert("RGB")
        tensor = torch.from_numpy(np.array(image, dtype=np.uint8))  # (H, W, 3)
        tensor = tensor.permute(2, 0, 1).float() / 255.0            # (3, H, W)
        return tensor, label


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


def evaluate(model: nn.Module, loader: DataLoader, device: torch.device) -> tuple[float, float]:
    model.eval()
    criterion = nn.CrossEntropyLoss()
    total_loss = 0.0
    correct = 0
    total = 0

    with torch.no_grad():
        for inputs, labels in loader:
            inputs = inputs.to(device)
            labels = labels.to(device)

            logits = model(inputs)
            loss = criterion(logits, labels)

            total_loss += loss.item() * inputs.size(0)
            preds = logits.argmax(dim=1)
            correct += (preds == labels).sum().item()
            total += inputs.size(0)

    return total_loss / total, correct / total


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    train_root = args.data_dir / "train"
    val_root = args.data_dir / "val"

    train_ds = ShapesDataset(train_root)
    val_ds = ShapesDataset(val_root)

    if train_ds.class_to_idx != val_ds.class_to_idx:
        raise ValueError("Train and validation class folders do not match")

    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=torch.cuda.is_available(),
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=torch.cuda.is_available(),
    )

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = SmallCNN(num_classes=len(train_ds.class_to_idx)).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.CrossEntropyLoss()

    print(f"Using device: {device}")
    print(f"Train samples: {len(train_ds)}, Val samples: {len(val_ds)}")

    for epoch in range(1, args.epochs + 1):
        model.train()
        running_loss = 0.0
        correct = 0
        total = 0

        for inputs, labels in train_loader:
            inputs = inputs.to(device)
            labels = labels.to(device)

            optimizer.zero_grad()
            logits = model(inputs)
            loss = criterion(logits, labels)
            loss.backward()
            optimizer.step()

            running_loss += loss.item() * inputs.size(0)
            preds = logits.argmax(dim=1)
            correct += (preds == labels).sum().item()
            total += inputs.size(0)

        train_loss = running_loss / total
        train_acc = correct / total
        val_loss, val_acc = evaluate(model, val_loader, device)

        print(
            f"Epoch {epoch:02d}/{args.epochs} | "
            f"train loss {train_loss:.4f} acc {train_acc:.3f} | "
            f"val loss {val_loss:.4f} acc {val_acc:.3f}"
        )

    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.out_dir / "model.pt"
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "class_to_idx": train_ds.class_to_idx,
        },
        out_path,
    )

    meta_path = args.out_dir / "classes.json"
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(train_ds.class_to_idx, f, indent=2)

    print(f"Saved model to: {out_path}")
    print(f"Saved classes to: {meta_path}")


if __name__ == "__main__":
    main()
