# About

Lego NXT2 C/C++ FW communicating with Raspberry Pi C++/Python SW.

Log-term goal: Raspberry Pi solving a Rubik's cube via Python/OpenCV and then controlling the NXT to physically solve it.

HIGHLY WORK-IN-PROGRESS, CODE NOT BUILDING YET

Unfortunately I only have Lego NXTs and don't want to buy a new set. However, ARM low-level programming is more fun for
me. So I experiment with own bare-metal firmware on the NXT (for sensor/actuator management) and application software
on the RPi (for the algorithms).

# Prerequisites

I'm using Ubuntu 20 as development platform. The embedded cross-build environment can be caught via Docker image below.

## General requirements

Install host build environment

````
sudo apt install git cmake gcc g++
````

Install required libraries

````
sudo apt install libusb-1.0-0-dev
````

# Build

## Docker

### Install Docker

Run install:

````
sudo apt install docker.io
````

Setup privileges for socket:

````
sudo usermod -aG docker $USER
````

### Build Image

Run setup script:

````
cd docker
./setup.sh
````

### Run Image

Run the container:

````
docker run --name rpi-nxt2 -it -v <root>/rpi-nxt2:/home/rpi-nxt2 ubuntu20-gcc-arm-none-eabi:latest
````

To re-run after exit:

````
docker start rpi-nxt2 -i
````

### Build ARM7 Firmware 

Run (in container):

````
cd /home/rpi-nxt2
scripts/build_nxt.sh <build_type> <app_name>
````

### Build Linux Software

Run:

````
cd /home/rpi-nxt2
scripts/build_linux.sh <build_type>
````

# Debugging

For debugging on the NXT I use the SEGGER J-Link ICE (EDU version). For that I soldered pins of the NXT board to expose
the JTAG interface. Via adapter, I then connect the ICE to the NXT.

## Debugging the FW

### Install J-Link

The Debian package can be downloaded from https://www.segger.com/downloads/jlink.

### Run GDB Server

TODO

### Attach to GDB Server

TODO