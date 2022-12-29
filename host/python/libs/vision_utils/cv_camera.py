#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import cv2
import numpy as np


class Camera:
    def __init__(self, image_width, image_height):
        self._image_width = image_width
        self._image_height = image_height
        self._capture = None

    def __del__(self):
        self._capture.release()

    def open(self, source):
        self._capture = cv2.VideoCapture(source)

        if not self._capture.isOpened():
            print("Cannot open capture device")
            return False

        print("Camera image size: {}x{}".format(
            int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))))

    def isOpen(self):
        self._capture.isOpened()

    def read(self):
        ret, frame = self._capture.read()
        frame = cv2.resize(frame, (self._image_width, self._image_height))
        if ret:
            return frame
        return None

    @staticmethod
    def exited():
        return cv2.waitKey(1) & 0xFF == ord('q')


if __name__ == '__main__':
    camera = Camera(640, 480)
    camera.open(0)

    while True:
        image = camera.read()
        if image is not None:
            cv2.imshow('original', image)
        if camera.exited():
            break
