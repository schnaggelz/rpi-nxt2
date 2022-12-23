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


class Camera(threading.Thread):

    def __init__(self, width=1640, height=1232, framerate=30, callback=None):
        super().__init__()
        self._camera = None
        self._main_size = (width, height)
        self._lores_size = (320, 240)
        self._framerate = framerate
        self._callback = callback
        self._stop_flag = False
        self._is_open = False

    def open(self):
        self._camera = picam2.Picamera2()
        video_config = self._camera.create_video_configuration(
            main={"size": self._main_size, "format": "XRGB8888"},
            lores={"size": self._lores_size, "format": "YUV420"})

        self._camera.configure(video_config)
        self._is_open = True

    def close(self):
        if not self.is_open():
            return

        self._camera.close()
        self._is_open = False

    def is_open(self):
        if self._camera is not None:
            if self._is_open:
                return True

        return False

    def run(self):
        self._camera.start()
        try:
            while not self._stop_flag:
                array = self._camera.capture_array()
                if self._callback:
                    self._callback(array)
        finally:
            self._camera.stop()

    def start(self):
        if not self.is_open():
            return

        self._stop_flag = False

        super().start()

    def stop(self):
        self._stop_flag = True

        super().join()

    def read(self):
        if not self.is_open():
            return

        return self._camera.capture_array()


if __name__ == '__main__':
    import fps_calcularor as fps

    counter = 0
    fps_calc = fps.FpsCalculator()

    def receive(image):
        global counter

        current_fps = fps_calc()
        if counter % 50 == 0:
            print("{}: {} fps".format(str(image.shape), current_fps))

        counter += 1

    camera = Camera(width=1640,
                    height=1232,
                    framerate=10,
                    callback=receive)
    camera.open()
    camera.start()

    try:
        camera.join()
    except KeyboardInterrupt:
        camera.stop()
    
    camera.close()
