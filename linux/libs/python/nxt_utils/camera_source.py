#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import cv2
import numpy as np


class CameraSource:
    def __init__(self, image_width, image_height):
        self.image_width = image_width
        self.image_height = image_height
        self.capture = None

    def __del__(self):
        self.capture.release()

    def open(self, source):
        self.capture = cv2.VideoCapture(source)

        if not self.capture.isOpened():
            print("Cannot open capture device")
            return False

        print("Camera image size: {}x{}".format(
            int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))))

    def isOpen(self):
        self.capture.isOpened()

    def read(self):
        ret, frame = self.capture.read()
        frame = cv2.resize(frame, (self.image_width, self.image_height))
        if ret:
            return frame
        return None

    @staticmethod
    def exited():
        return cv2.waitKey(1) & 0xFF == ord('q')


if __name__ == '__main__':
    source = CameraSource(640, 480)
    source.open(0)

    while True:
        image = source.read()
        if image is not None:
            cv2.imshow('original', image)
        if source.exited():
            break
