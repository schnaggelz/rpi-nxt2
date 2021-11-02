# Remove/stop container/image
docker stop rpi-nxt2
docker rm rpi-nxt2
docker rmi ubuntu20-gcc-arm-none-eabi

# Build our image
docker build --tag ubuntu20-gcc-arm-none-eabi .
