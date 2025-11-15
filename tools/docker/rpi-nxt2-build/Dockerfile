FROM ubuntu:22.04

ENV TZ=Europe/Germany
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update
RUN apt install -y tzdata sudo curl wget
RUN apt install -y build-essential gcc g++ make cmake git
RUN apt install -y gcc-arm-none-eabi
RUN apt install -y python3-pybind11
RUN apt install -y libpython3-dev libusb-1.0-0-dev libncurses5-dev
RUN apt install -y usbutils

# Create ubuntu user with sudo privileges
RUN useradd -u 1000 -ms /bin/bash ubuntu && usermod -aG sudo ubuntu
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ubuntu

WORKDIR /home/ubuntu
