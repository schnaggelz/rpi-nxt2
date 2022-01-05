#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

from picamera import PiCamera
from picamera.array import PiRGBArray

class CameraSource:
    def __init__(self, image_width, image_height, read_callback=None):
        self._image_width = image_width
        self._image_height = image_height
        self._callback = read_callback

    def open(self):
        self._camera = PiCamera()
        self._camera.resolution = (self._image_width, self._image_height)
        self._buffer = PiRGBArray(self._camera,
                                  size=(self._image_width, self._image_height))

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
        return self._camera is not None

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

        #print('Captured %dx%d image' % (
        #    image.shape[1], image.shape[0]))

    camera = CameraSource(640, 480, receive)
    camera.open()
    camera.run()
