#!/bin/bash

docker run --user $(id -u):$(id -g) -it -v $(pwd)/../../..:/workspace rpi-nxt2:latest
