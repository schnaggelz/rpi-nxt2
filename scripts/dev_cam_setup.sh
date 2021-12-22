#!/bin/bash

# On Raspberry PI to stream to dev PC
libcamera-vid -t 0 --inline --listen -o tcp://0.0.0.0:8888

# On dev PC to stream to v4l-loopback for easy access via device
ffmpeg -f h264 -i tcp://raspina:8888 -f v4l2 -pix_fmt yuv420p /dev/video0

