# Install Pytorch Based ML Environment

## Install ROCM

I use Distrox. See ``distrobox-ubuntu-rocm.ini``.

### Check Installation

```sh
rocm-smi
```

## Install Pytorch (ROCm)

In separate venv

```sh
python3 -m venv ~/.venv-pytorch-rocm
source ~/.venv-pytorch-rocm/bin/activate
pip3 install wheel setuptools
pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/rocm7.2
pip3 install onnxscript
```

### Check Installation

For my local setup:

```sh
python3 ./check_cpu_ml_infra.py
```

## Install Pytorch (CPU only)

```sh
python3 -m venv ~/.venv-pytorch-rocm
source ~/.venv-pytorch-rocm/bin/activate
pip3 install wheel setuptools
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip3 install onnxscript
```

For my local setup:

```sh
python3 ./check_cpu_ml_infra.py
```

## Install Label Studio

In separate venv

```sh
python3 -m venv ~/.venv-label-studio
source ~/.venv-label-studio/bin/activate
pip3 install ultralytics label-studio onnx onnxsim
```
