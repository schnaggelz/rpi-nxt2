# About

Lego NXT2 C/C++ FW communicating with Raspberry Pi C++/Python SW.

Log-term goal: Raspberry Pi solving a Rubik's cube via Python/OpenCV and then controlling the NXT to physically solve it.

HIGHLY WORK-IN-PROGRESS, CODE NOT BUILDING YET

Unfortunately I only have Lego NXTs and don't want to buy a new set. However, ARM low-level programming is more fun for
me. So I experiment with own bare-metal firmware on the NXT (for sensor/actuator management) and application software
on the RPi (for the algorithms).

Most of the code is coming from my Bitbucket repo at https://bitbucket.org/schnaggelz/nxt/src/master. I decided to move
to Github for upcoming experiments.

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

### Run Build Scripts

### ARM7 FW Build

Run (in container):

````
cd /home/rpi-nxt2
scripts/build_nxt.sh <build_type> <app_name>
````

### Linux SW Build

Run (in container):

````
cd /home/rpi-nxt2
scripts/build_linux.sh <build_type>
````

