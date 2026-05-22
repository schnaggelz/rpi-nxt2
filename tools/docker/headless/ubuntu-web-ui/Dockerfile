FROM ghcr.io/open-webui/open-webui:main

# The base image is Debian-based, so we can use apt to install useful utilities.
# We run as root by default in this image step anyway.
RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    curl \
    wget \
    htop \
    procps \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*