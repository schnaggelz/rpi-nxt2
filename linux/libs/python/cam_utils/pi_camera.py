#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import time

from picamera import PiCamera
from picamera.array import PiRGBArray


class Camera:
    def __init__(self, image_width, image_height, framerate=30, read_callback=None):
        self._image_width = image_width
        self._image_height = image_height
        self._framerate = framerate
        self._callback = read_callback

    def open(self):
        time.sleep(0.5)

        self._camera = PiCamera(resolution=(self._image_width, self._image_height),
                                framerate=self._framerate)
        #self._camera.exposure_mode = 'night'
        self._buffer = PiRGBArray(self._camera,
                                  size=(self._image_width, self._image_height))

    def close(self):
        if not self.is_open():
            return

        self._camera.close()

    def run(self):
        if not self.is_open():
            return

        for frame in self._camera.capture_continuous(self._buffer,
                                                     format='bgr',
                                                     use_video_port=True):
            if self._callback:
                self._callback(frame.array)

            self._buffer.truncate(0)

    def is_open(self):
        if self._camera is not None:
            return not self._camera.closed
        return False

    def read(self):
        if not self.is_open():
            return

        self._buffer.truncate(0)
        self._camera.capture(self._buffer, 'bgr')

        frame = self._buffer.array

        return frame


if __name__ == '__main__':
    def receive(image):
        pass

        # print('Captured %dx%d image' % (
        #    image.shape[1], image.shape[0]))


    camera = Camera(image_width=640,
                    image_height=480,
                    framerate=10,
                    read_callback=receive)
    camera.open()
    camera.run()
