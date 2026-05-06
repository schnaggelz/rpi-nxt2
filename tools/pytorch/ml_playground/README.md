# ML Playground — Track Dataset (PyTorch + ROCm)

Generates synthetic track images: thick, mostly black lines at arbitrary angles
on a light background, for line-following / steering-angle experiments.

## Structure

- `generate_dataset.py`: generate the track dataset
- `train_classifier.py`: train SmallCNN and save `outputs/model.pt`
- `capture_dataset.py`: capture labeled webcam photos on key press
- `webcam_classify.py`: run live webcam inference with `outputs/model.pt`
- `export_onnx.py`: export `outputs/model.pt` to `outputs/model.onnx` (PyTorch venv)
- `requirements.txt`: Python dependencies

## Generate Synthetic Dataset

```bash
python generate_dataset.py --out-dir data --train-count 400
```

From repo root:

```bash
python tools/pytorch/ml_playground/generate_dataset.py \
  --out-dir=tools/pytorch/ml_playground/data \
  --train-count=1000
```

Synthetic classes: `track`, `crossing`, `sign`.

### Hybrid mode (synthetic train + real val)

The generator now creates only `train/` synthetic images, so your real `val/`
images are not overwritten.

Use `--clean` only when you intentionally want to delete existing data in `--out-dir` first.

Output layout, e.g. for `track`:

```
data/
  train/
    track/          ← PNG images
    track_labels.csv
```

Each CSV row contains: `image`, `class_name`, `angle_degrees`, `thickness_px`,
`offset_px`, `track_color`.

### All options

| Flag            | Default       | Description                        |
|-----------------|---------------|------------------------------------|
| `--out-dir`     | `data`        | Output root directory              |
| `--image-size`  | `64`          | Square image side length in pixels |
| `--train-count` | `400`         | Number of training images          |
| `--seed`        | `42`          | Random seed                        |
| `--clean`       | `false`       | Remove output directory before generation |

## Train Model

```bash
python train_classifier.py --data-dir data --epochs 20 --out-dir outputs
```

And from repo root:

```bash
python tools/pytorch/ml_playground/train_classifier.py \
  --data-dir=tools/pytorch/ml_playground/data \
  --out-dir=tools/pytorch/ml_playground/outputs \
  --epochs 20
```

Trains a `SmallCNN` classifier on image folders using an explicit split:

- `data/train/<class>/*`
- `data/val/<class>/*`

Supported image formats: PNG, JPG/JPEG, BMP, WEBP.

Outputs:

- `outputs/model.pt` — PyTorch checkpoint (`model_state_dict` + `class_to_idx`)
- `outputs/classes.json` — class-name-to-index mapping

### All options

| Flag           | Default   | Description                                   |
|----------------|-----------|-----------------------------------------------|
| `--data-dir`   | `data`    | Dataset root containing `train/` and `val/` |
| `--epochs`     | `10`      | Number of training epochs                     |
| `--batch-size` | `64`      | Batch size                                    |
| `--lr`         | `1e-3`    | Learning rate (Adam)                          |
| `--workers`    | `2`       | DataLoader worker processes                   |
| `--out-dir`    | `outputs` | Directory for saved model & metadata          |
| `--cpu`        | `false`   | Force CPU training even if CUDA is available  |
| `--image-size` | `64`      | Resize inputs to a square side length         |

Training prints total elapsed wall-clock time at the end.

## Live Webcam Classification

Run from the playground directory:

```bash
python webcam_classify.py
```

From repo root:

```bash
python tools/pytorch/ml_playground/webcam_classify.py \
  --model tools/pytorch/ml_playground/outputs/model.pt
```

Discover available cameras first:

```bash
python webcam_classify.py --list-cameras
```

Then pick one explicitly:

```bash
python webcam_classify.py --camera 1
```

Press `q` in the preview window to quit.

### Webcam options

| Flag             | Default            | Description                              |
|------------------|--------------------|------------------------------------------|
| `--model`        | `outputs/model.pt` | Path to trained PyTorch checkpoint       |
| `--camera`       | `0`                | Camera index to open                     |
| `--size`         | `64`               | Resize input frame to this square size   |
| `--cpu`          | `false`            | Force CPU inference even if CUDA exists  |
| `--list-cameras` | `false`            | Probe cameras, print detected devices, and exit |

## Webcam Data Capture

Capture training images from webcam with keyboard control.

Run from repo root:

```bash
python tools/pytorch/ml_playground/capture_dataset.py \
  --camera 0 \
  --label track \
  --out-dir tools/pytorch/ml_playground/data \
  --prefix img
```

Controls:

- `Space`: capture one photo
- `q`: quit

Output:

- Images are saved under `out-dir/label/` as JPG files
- A capture manifest is appended to `out-dir/captures.csv`

### Capture options

| Flag         | Default       | Description                              |
|--------------|---------------|------------------------------------------|
| `--out-dir`  | `data`        | Root output directory                    |
| `--label`    | `unlabeled`   | Class label (used as subfolder name)     |
| `--camera`   | `0`           | Camera index                             |
| `--prefix`   | `img`         | Prefix for generated image filenames     |


## Export ONNX

```bash
python export_onnx.py --model outputs/model.pt --out-dir outputs --image-size 64
```

And from repo root:

```bash
python tools/pytorch/export_onnx.py \
  --model tools/pytorch/ml_playground/output/model.pt \
  --out-dir tools/pytorch/ml_playground/outputs \
  --image-size 64
```

## Notes

- Uses `torch.cuda.is_available()` to pick GPU (ROCm appears as CUDA in PyTorch APIs).
- The angle label enables regression / steering-angle training in addition to classification.

