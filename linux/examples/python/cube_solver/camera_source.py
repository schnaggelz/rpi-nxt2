#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import cv2


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
    def exited():
        return cv2.waitKey(1) & 0xFF == ord('q')


if __name__ == '__main__':
    address = "http://192.168.242.105:4747/video"
    cs = CameraSource(address, 1280, 720)
    cs.open()

    while True:
        image = cs.read()
        if image is not None:
            cv2.imshow('original', image)
        if cs.exited():
            break
