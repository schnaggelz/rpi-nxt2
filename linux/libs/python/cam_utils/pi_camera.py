#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import time
import numpy as np

from picamera import PiCamera
from picamera.array import PiRGBArray
from picamera.array import PiRGBAnalysis

class Camera:

    class FrameProcessor(PiRGBAnalysis):
        def __init__(self, camera, callback):
            super(Camera.FrameProcessor, self).__init__(camera)
            self._callback = callback
            self._counter = 0

        @property
        def counter(self):
            return self._counter

        def analyze(self, array):
            if self._callback:
                self._callback(array)

    def __init__(self, width=1640, height=1232, framerate=30, callback=None):
        self._width = width
        self._height = height
        self._framerate = framerate
        self._callback = callback
        self._processor = None
        self._stop = True

    def open(self):
        time.sleep(0.5)

        self._camera = PiCamera(resolution=(self._width, self._height),
                                framerate=self._framerate)
        # camera.awb_mode = 'off'
        # camera.awb_gains = (1.4, 1.5)

        self._processor = self.FrameProcessor(self._camera, self._callback)

    def close(self):
        if not self.is_open():
            return

        self._camera.close()

    def run(self):
        if not self.is_open():
            return

        self._stop = False

        self._camera.start_recording(self._processor, 'bgr')
        try:
            while not self._stop:
                self._camera.wait_recording(1)
        except KeyboardInterrupt:
            pass
        finally:
            self._camera.stop_recording()

    def stop(self):
        self._stop = True

    def is_open(self):
        if self._camera is not None and self._processor is not None:
            return not self._camera.closed
        return False

    def read(self):
        if not self.is_open():
            return

        array = np.empty((self._height, self._width), dtype=np.uint8)
        self._camera.capture(array, 'bgr')

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
    camera.run()
