# Install Hailo SDK

## Hailo Data Flow Compiler

In separate venv

````sh
python3 -m venv ~/.venv-hailo-sdk
source ~/.venv-hailo-sdk/bin/activate
pip3 install tk
pip3 install ~/tmp/hailo_dataflow_compiler-*.whl
````

## Check Installation

First usage of 

````sh
python3 ./check_hailo_dfc_infra.py
````

should generate a requirements table like this:


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

Check if anything is missing.

# Run Model Compilation

In the Hailo SDK venv:

```bash
python3 ./compile_hailo.py --onnx outputs/model.onnx --data-dir data/tracks --out-dir ./outputs --image-size 64
```
