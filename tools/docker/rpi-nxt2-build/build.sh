#!/bin/bash

# Remove/stop container/image
docker stop rpi-nxt2
docker rm rpi-nxt2
docker rmi rpi-nxt2-build

# Build our image
docker build --tag rpi-nxt2 .
