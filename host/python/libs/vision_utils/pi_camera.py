#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import time
import numpy as np
import threading

from picamera import PiCamera
from picamera.array import PiRGBArray
from picamera.array import PiRGBAnalysis


class Camera(threading.Thread):
    class FrameProcessor(PiRGBAnalysis):
        def __init__(self, camera, callback):
            super().__init__(camera)
            self.__callback = callback
            self.__counter = 0

        @property
        def counter(self):
            return self.__counter

        def analyze(self, array):
            if self.__callback:
                self.__callback(array)

    def __init__(self, width=1640, height=1232, framerate=30, callback=None):
        super().__init__()
        self.__width = width
        self.__height = height
        self.__framerate = framerate
        self.__callback = callback
        self.__processor = None
        self.__stop_flag = False

    def open(self):
        time.sleep(0.5)

        self.__camera = PiCamera(resolution=(self.__width, self.__height),
                                 framerate=self.__framerate)
        # self.__camera.awb_mode = 'off'
        # self.__camera.awb_gains = (1.2, 1.3)
        self.__camera.video_denoise = True
        self.__camera.image_effect = 'denoise'

        self.__processor = self.FrameProcessor(self.__camera, self.__callback)

    def close(self):
        if not self.is_open():
            return

        self.__camera.close()

    def run(self):
        self.__camera.start_recording(self.__processor, 'bgr')
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

        array = np.empty((self.__height, self.__width), dtype=np.uint8)
        self.__camera.capture(array, 'bgr')

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
