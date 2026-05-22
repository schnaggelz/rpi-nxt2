# ROCm ML Development Environment

A [Distrobox](https://distrobox.it/) container based on Ubuntu with AMD ROCm support for ML development.

## Prerequisites

- [Distrobox](https://distrobox.it/) installed on the host
- AMD GPU with ROCm-compatible drivers (`/dev/dri` and `/dev/kfd` accessible)
- Podman or Docker as the container backend

## Create the container

```bash
distrobox assemble create --file distrobox-ubuntu-rocm.ini
```

> **Note:** The name is fixed to `ubuntu-rocm` (set via `image_name` in the `.ini` file).
> To use a custom name, create the container manually instead:

```bash
distrobox create \
  --image ubuntu:latest \
  --name my-custom-name \
  --init \
  --home "$HOME/.distrobox/my-custom-name" \
  --additional-flags "--device /dev/dri --device /dev/kfd -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY"
```

## Enter the container

```bash
distrobox enter ubuntu-rocm
```

## What gets installed

The `init_hooks` in the `.ini` file automatically install ROCm inside the container on first start:

1. Downloads the `amdgpu-install` package from the ROCm 7.2 repository
2. Installs it and updates the package lists
3. Installs the full `rocm` meta-package

Additional packages: `git`, `wget`, `curl`, `htop`, `cmake`, `byobu`, `zsh`, `python3`, `pip`, `venv`, and more.

## Verify ROCm

After entering the container, verify the installation:

```bash
rocminfo
rocm-smi
```

## Notes

- `nvidia=false` — ROCm (AMD) is used instead of CUDA
- The container home directory is isolated at `~/.distrobox/ubuntu-rocm`
- Display forwarding (X11 and Wayland) is configured via environment variables
