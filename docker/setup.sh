# Remove/stop container/image
docker stop rpi-nxt2
docker rm rpi-nxt2
docker rmi ubuntu20-gcc-arm-none-eabi

# Build our image
docker build --tag ubuntu20-gcc-arm-none-eabi .

# Run the container
docker run --name nxt -it -v ~/Develop/rpi-nxt2:/home/rpi-nxt2 ubuntu20-gcc-arm-none-eabi:latest

# To re-run after exit
#docker start nxt -i
