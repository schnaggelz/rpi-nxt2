# Python Rubik's Cube Solver

This Python application will use my NXT remote control library with its Python binding `nxt_remote_py` to control the
Lego model gathered from the MindCuber page (see http://mindcuber.com/).

![My Cuber](../../../../doc/cuber.jpg)

The example uses the Python package `kociemba` to solve the cube. It is console only, no graphical subsystem is
required on the Raspberry Pi!

## Lego Building Instructions

See [MindCuber Build Instructions](http://mindcuber.com/mindcuber/MindCuber.pdf).

The model was modified to hold the Camera and the Raspberry Pi 3B and the Raspberry Pi Camera V2. For the mini
computer and the camera module I've built cases out of classic logo building blocks. The camera stand is sitting
on top of the color sensor which I still kept for a simple program later-on.

## Structure

The example consists of:

- a machine representation class `SolverMachine`
- a cube detector class `ColorDetector`
- a console view class `SolverConsole` that can be used via SSH
- video utility classes (camera, stream receiver, stream sender, ...)
- general utility classes (periodic timer, console window)

## Debugging

### Video Streaming

The image detection can be viewed remotely via the class `VideoReceiver`. 

### Video Streaming plus Color Range Adjustment

For proper calibration, the class `ColorRanger` can also receive the image in order to be able to adapt the color
ranges.

## Deployment

Put Python module into the `PYTHONPATH`, e.g. when the binaries are deployed via CMake into `~/.local/lib`, like 
assumed in [pi_setup.sh](../../../../scripts/pi_setup.sh):

````
export PYTHONPATH=$SOURCE_DIR/linux/libs/python:$HOME/.local/lib
````

#### Runtime requirements (e.g. on Raspberry OS)

Install required libraries

````
sudo apt install python3
sudo apt install python3-pip
sudo apt install python3-opencv
sudo apt install python3-zmq
sudo apt-get install libffi-dev
pip install kociemba
````

To speed up OpenCV and generate a minimal deployment, have a look at
[opencv-lite-on-raspberry-pi](https://qengineering.eu/install-opencv-lite-on-raspberry-pi.html).
