# Pytorch based ML Environment

## Install ROCM

I use Distrox. See ``distrobox-ubuntu-rocm.ini``.

### Check Installation

````sh
rocm-smi
````

## Install Pytorch

In separate venv

````sh
python3 -m venv ~/.venv-rocm-pytorch  
source ~/.venv-rocm-pytorch/bin/activate
pip3 install wheel setuptools
pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/rocm7.2
````

### Check Installation

For my local setup:

````sh
python3 ./check_rocm_ml_infra.py
````

## Label Studio

In separate venv

````sh
python3 -m venv ~/.venv-label-studio
source ~/.venv-label-studio/bin/activate
pip3 install ultralytics label-studio onnx onnxsim
````

