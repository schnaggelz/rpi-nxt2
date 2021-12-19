#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import camera_source as cs
import numpy as np
from imutils import contours
import cv2


class FrameDetector:
    COLORS = {
        'gray': ([76, 0, 41], [179, 255, 70]),  # Gray
        'blue': ([69, 120, 100], [179, 255, 255]),  # Blue
        'yellow': ([21, 110, 117], [45, 255, 255]),  # Yellow
        'orange': ([0, 110, 125], [17, 255, 255])  # Orange
    }

    def __init__(self, source_path):
        self.camera = cs.CameraSource(source_path, 640, 480)
        self.camera.open()

    def detect(self):
        image = self.camera.read()
        if image is None or self.camera.exited():
            return False

        original = image.copy()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = np.zeros(image.shape, dtype=np.uint8)

        open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        for color, (lower, upper) in FrameDetector.COLORS.items():
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            color_mask = cv2.inRange(image, lower, upper)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, close_kernel, iterations=5)

            color_mask = cv2.merge([color_mask, color_mask, color_mask])
            mask = cv2.bitwise_or(mask, color_mask)

        cv2.imshow('mask', mask)

        return True


if __name__ == '__main__':
    address = "http://192.168.242.105:4747/video"
    address = "/home/timon/Pictures/vlcsnap-2021-12-19-13h26m32s813.png"
    fd = FrameDetector(address)

    # while True:
    res = fd.detect()
        # if not res:
        #     break

    cv2.waitKey()
