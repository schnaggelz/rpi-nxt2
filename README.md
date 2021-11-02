# About

Lego NXT2 C/C++ FW communicating with Raspberry Pi C++/Python SW.

HIGHLY WORK-IN-PROGRESS, CODE NOT BUILDING YET

Unfortunately I only have Lego NXTs and don't want to buy a new set. However, ARM low-level programming is more fun for
me. So I experiment with own bare-metal firmware on the NXT (for sensor/actuator management) and application software
on the RPi (for the algorithms).

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
docker run --name rpi-nxt2 -it -v ~/Develop/rpi-nxt2:/home/rpi-nxt2 ubuntu20-gcc-arm-none-eabi:latest
````

To re-run after exit:

````
docker start rpi-nxt2 -i
````

