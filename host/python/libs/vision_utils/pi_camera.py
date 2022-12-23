#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import time
import numpy as np
import threading
import picamera2 as picam2

from picamera2 import encoders as cam_encoders
from picamera2 import outputs as cam_outputs

class Camera(threading.Thread):

    def __init__(self, width=1640, height=1232, framerate=30, callback=None):
        super().__init__()
        self.__size = (width, height)
        self.__lores_size = (320, 240)
        self.__framerate = framerate
        self.__callback = callback
        self.__processor = None
        self.__stop_flag = False

    def open(self, lsize = (320, 240)):
        time.sleep(0.5)

        self.__camera = picam2.PiCamera2()
        video_config = self.__camera.create_video_configuration(
            main={"size": self.__size, "format": "XRGB8888"},
            lores={"size": self.__lores_size, "format": "YUV420"})
        self.__camera.configure(video_config)

        encoder = cam_encoders.H264Encoder(1000000, repeat=True)
        output = cam_outputs.CircularOutput()
        encoder.output = [output]

        self.__camera.encoder = encoder


    def close(self):
        if not self.is_open():
            return

        self.__camera.close()

    def run(self):
        self.__camera.start()
        self.__camera.start_encoder()

        self.__processor = self.FrameProcessor(self.__camera, self.__callback)
        try:
            while not self.__stop_flag:
                self.__camera.wait_recording(1)
        finally:
            self.__camera.stop_recording()

    def start(self):
        if not self.is_open():
            return

        self.__stop_flag = False

        super().start()

    def stop(self):
        self.__stop_flag = True

        super().join()

    def is_open(self):
        if self.__camera is not None and self.__processor is not None:
            return not self.__camera.closed

        return False

    def read(self):
        if not self.is_open():
            return
        
        array = self.__camera.picam2.capture_array()

        return array


if __name__ == '__main__':
    last_time = time.time()
    counter = 0


    def receive(image):
        global last_time
        global counter

        elapsed_time = time.time() - last_time
        secs_elapsed = elapsed_time % 60
        fps = 1 / secs_elapsed

        if counter % 50 == 0:
            print("{}: {} fps".format(str(image.shape), fps))

        counter += 1
        last_time = time.time()


    camera = Camera(width=640,
                    height=480,
                    framerate=10,
                    callback=receive)
    camera.open()
    camera.start()

    camera.join()
