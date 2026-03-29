# Hailo AI Kit Setup

## Data Flow Compiler

Download from [Hailo Dev Zone](https://hailo.ai/developer-zone/software-downloads/?product=ai_accelerators&device=hailo_8_8l).

```bash

# Install dependencies
sudo apt install -y libgraphviz-dev graphviz

# Then in the (CUDA/Pytorch) ML venv
pip3 install onnxscript tk
pip3 install ~/tmp/hailo_dataflow_compiler-*.whl

# Check if installed 
python3 -c "import hailo_sdk_client; print(hailo_sdk_client.__version__)"

# or
hailo --version
```
