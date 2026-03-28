import sys, importlib.util
print('python_executable:', sys.executable)

spec = importlib.util.find_spec('torch')
print('torch_installed:', bool(spec))

if not spec:
    raise SystemExit(0)

import torch

print('torch_version:', torch.__version__)
print('torch_cuda_is_available:', torch.cuda.is_available())
print('torch_cuda_device_count:', torch.cuda.device_count())
print('torch_version_cuda:', torch.version.cuda)
print('torch_version_hip:', getattr(torch.version, 'hip', None))

if torch.cuda.is_available() and torch.cuda.device_count() > 0:
    idx = 0
    print('torch_cuda_device_0_name:', torch.cuda.get_device_name(idx))
    a = torch.tensor([1.0, 2.0, 3.0], device='cuda')
    b = a * 2
    print('torch_gpu_tensor_ok:', b.tolist())
else:
    print('torch_gpu_tensor_ok: False')
