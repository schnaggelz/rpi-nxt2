# About

Lego NXT2 C++ FW communicating with Raspberry Pi

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

