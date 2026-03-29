# ML Setup

## Host

### Pytorch for ROCM

In separate venv

````sh
python3 -m venv ~/.venv-rocm-pytorch  
source ~/.venv-rocm-pytorch/bin/activate
pip3 install wheel setuptools
pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/rocm7.2
````

### Hailo Data Flow Compiler

In separate venv

````sh
python3 -m venv ~/.venv-hailo-sdk
source ~/.venv-hailo-sdk/bin/activate
pip3 install tk
pip3 install ~/tmp/hailo_dataflow_compiler-*.whl
````

### Label Studio

In separate venv

````sh
python3 -m venv ~/.venv-label-studio
source ~/.venv-label-studio/bin/activate
pip3 install ultralytics label-studio onnx onnxsim
````


### Check infrastructure available

For my local setup:

````sh
python3 ./check_rocm_ml_infra.py
````

| Component       | Requirement     | Found  | Required    |
|-----------------|-----------------|--------|-------------|
| OS              | Ubuntu          | Ubuntu | Required    |
| Release         | 20.04           | 24.04  | Required    |
| Package         | python3-tk      | X      | Required    |
| Package         | graphviz        | V      | Required    |
| Package         | libgraphviz-dev | V      | Required    |
| Package         | python3.12-dev  | V      | Required    |
| RAM(GB)         | 16              | 120    | Required    |
| RAM(GB)         | 32              | 120    | Recommended |
| CPU-Arch        | x86_64          | x86_64 | Required    |
| CPU-flag        | avx             | V      | Required    |
| Var:CC          | unset           | unset  | Required    |
| Var:CXX         | unset           | unset  | Required    |
| Var:LD          | unset           | unset  | Required    |
| Var:AS          | unset           | unset  | Required    |
| Var:AR          | unset           | unset  | Required    |
| Var:LN          | unset           | unset  | Required    |
| Var:DUMP        | unset           | unset  | Required    |
| Var:CPY         | unset           | unset  | Required    |
