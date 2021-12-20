#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).
import time
from collections import defaultdict

import camera_source as cs
import cube_colors as cc
import numpy as np
import cv2


class FrameDetector:
    def __init__(self, source_path, profile):
        self.profile = profile
        self.threshold_area = 5000
        self.camera = cs.CameraSource(source_path, 640, 480)
        self.camera.open()

    def contours(self, hsv_frame):
        color_ctrs = defaultdict(list)
        for color, ranges in cc.CubeColors.ranges(self.profile).items():
            mask = None
            for range in ranges:
                r_min = np.array(range[0], np.uint8)
                r_max = np.array(range[1], np.uint8)
                if mask is None:
                    mask = cv2.inRange(hsv_frame, r_min, r_max)
                else:
                    mask |= cv2.inRange(hsv_frame, r_min, r_max)

            canny = cv2.Canny(mask, 50, 100)
            ctrs, h = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for ctr in ctrs:
                rect = cv2.minAreaRect(ctr)
                width = rect[1][0]
                height = rect[1][1]
                if width > 100 and height > 100:
                    color_ctrs[color].append(ctr)

        return color_ctrs

    def detect(self):
        frame = self.camera.read()
        if frame is None or self.camera.exited():
            return False

        bf_frame = cv2.bilateralFilter(frame, 9, 75, 75)
        hsv_frame = cv2.cvtColor(bf_frame, cv2.COLOR_BGR2HSV)

        cntrs = self.contours(hsv_frame)

        for color, cntrs in cntrs.items():
            for cntr in cntrs:
                x, y, w, h = cv2.boundingRect(cntr)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                frame = cv2.putText(frame, color, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("solver", frame)

        return True


if __name__ == '__main__':

    # TODO cmd line args
    src = '/dev/video0'
    prf = 'niko'

    fd = FrameDetector(src, prf)

    while True:
        res = fd.detect()
        if not res:
            break

    # cv2.waitKey()
