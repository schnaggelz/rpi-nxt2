FROM ubuntu:18.04

ARG USER_ID
ARG GROUP_ID

# Add user to sudoers
RUN apt-get update && apt-get install -y sudo
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
RUN adduser user sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER user

# Install SDK deps
RUN sudo apt-get update && sudo apt-get install -y \
    libgconf-2-4 \
    locales \
    netcat \
    openssh-server \
    usbutils \
    qemu-user-static

COPY sdkmanager_amd64.deb /
RUN sudo apt-get install -y /sdkmanager_amd64.deb
