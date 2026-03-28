# ML Playground — Track Dataset (PyTorch + ROCm)

Generates synthetic track images: thick, mostly black lines at arbitrary angles
on a light background, for line-following / steering-angle experiments.

## Structure

- `generate_dataset.py`: generate the track dataset
- `requirements.txt`: Python dependencies

## Quick start

```bash
python generate_dataset.py \
  --out-dir data/tracks \
  --train-count 400 \
  --val-count 100 \
  --track-width-min 8 \
  --track-width-max 18
```

Output layout:

```
data/tracks/
  train/
    track/          ← PNG images
    track_labels.csv
  val/
    track/
    track_labels.csv
```

Each CSV row contains: `image`, `class_name`, `angle_degrees`, `thickness_px`,
`offset_px`, `track_color`.

## All options

| Flag | Default | Description |
|---|---|---|
| `--out-dir` | `data/tracks` | Output root directory |
| `--image-size` | `64` | Square image side length in pixels |
| `--train-count` | `400` | Number of training images |
| `--val-count` | `100` | Number of validation images |
| `--track-width-min` | `8` | Minimum track thickness in pixels |
| `--track-width-max` | `18` | Maximum track thickness in pixels |
| `--black-track-prob` | `0.85` | Probability the track is pure black |
| `--seed` | `42` | Random seed |

## Notes

- Uses `torch.cuda.is_available()` to pick GPU (ROCm appears as CUDA in PyTorch APIs).
- The angle label enables regression / steering-angle training in addition to classification.
