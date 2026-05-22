FROM ubuntu:24.04

# Prevent interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# Environment variables from your distrobox config
ENV OLLAMA_HOST=0.0.0.0
ENV OLLAMA_MAX_VRAM=22528
ENV OLLAMA_NUM_PARALLEL=1

# Install additional packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates gnupg \
    zsh nano zstd git wget curl usbutils \
    python3-dev python3-pip python3-venv \
    libssl-dev libffi-dev build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install ROCm
RUN wget https://repo.radeon.com/amdgpu-install/7.2.3/ubuntu/noble/amdgpu-install_7.2.3.70203-1_all.deb \
    && apt-get update \
    && apt-get install -y ./amdgpu-install_7.2.3.70203-1_all.deb \
    && rm ./amdgpu-install_7.2.3.70203-1_all.deb \
    && apt-get update && apt-get install -y rocm \
    && rm -rf /var/lib/apt/lists/*

# Install Ollama
RUN curl -fsSL https://ollama.com/install.sh | sh

# Set the default shell to loop/wait or drop into zsh
CMD ["/bin/zsh"]
