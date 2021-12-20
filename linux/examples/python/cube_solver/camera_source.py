#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import cv2
import numpy as np


class CameraSource:
    def __init__(self, source_path, image_width, image_height):
        self.source_path = source_path
        self.image_width = image_width
        self.image_height = image_height
        self.capture = None

    def __del__(self):
        self.capture.release()

    def open(self):
        self.capture = cv2.VideoCapture(self.source_path)

        if not self.capture.isOpened():
            print("Cannot open capture device")
            return False

        cap_image_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        cap_image_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print("Camera image size: {}x{}".format(cap_image_width, cap_image_height))

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)

        print("Target image size: {}x{}".format(self.image_width, self.image_height))

    def is_open(self):
        self.capture.isOpened()

    def read(self):
        ret, frame = self.capture.read()
        # frame = cv2.resize(frame, (self.image_width, self.image_height))
        if ret:
            return frame
        return None

    @staticmethod
    def to_hsv(frame):
        tmp_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return np.array(tmp_frame)

    @staticmethod
    def exited():
        return cv2.waitKey(1) & 0xFF == ord('q')


if __name__ == '__main__':
    address = "/dev/video0"
    cs = CameraSource(address, 1280, 720)
    cs.open()

    while True:
        image = cs.read()
        if image is not None:
            cv2.imshow('original', image)
        if cs.exited():
            break
