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

    def __init__(self, main_size=(1640, 1232), lores_size=(320, 240), main_callback=None, lores_callback=None):
        super().__init__()
        self._camera = None
        self._main_size = main_size
        self._lores_size = lores_size
        self._main_callback = main_callback
        self._lores_callback = lores_callback
        self._stop_flag = False
        self._is_open = False

    def open(self):
        self._camera = picam2.Picamera2()
        video_config = self._camera.create_video_configuration(
            main={"size": self._main_size, "format": "RGB888"},
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
                
                if self._main_callback:
                    array = self._camera.capture_array("main")
                    self._main_callback(array)
                if self._lores_callback:
                    array = self._camera.capture_array("lores")
                    self._lores_callback(array)
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

    camera = Camera(lores_size=(640, 480),
                    lores_callback=receive)
    camera.open()
    camera.start()

    try:
        camera.join()
    except KeyboardInterrupt:
        camera.stop()
    
    camera.close()
