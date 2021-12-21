#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import camera_source as cs
import cube_colors as cc
import numpy as np
import cv2


class FrameDetector:
    class Box:
        __slots__ = ['color', 'contour']

        def __init__(self, color, contour):
            self.color = color
            self.contour = contour

    def __init__(self, source_path, profile):
        self.profile = profile
        self.threshold_area = 5000
        self.camera = cs.CameraSource(source_path, 640, 480)
        self.camera.open()

    @staticmethod
    def contourPrecedence(box, cols):
        tolerance = 10
        origin = cv2.boundingRect(box.contour)
        return ((origin[1] // tolerance) * tolerance) * cols + origin[0]

    @staticmethod
    def displayBoxes(img, boxes):
        for idx, box in enumerate(boxes):
            x, y, w, h = cv2.boundingRect(box.contour)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            text = "#{}:{}".format(idx, box.color)
            img = cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                              0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("solver", img)

    def getBoxes(self, img):
        min_size = 100
        boxes = []
        for color, ranges in cc.CubeColors.ranges(self.profile).items():

            mask = None
            for r in ranges:
                r_min = np.array(r[0], np.uint8)
                r_max = np.array(r[1], np.uint8)
                if mask is None:
                    mask = cv2.inRange(img, r_min, r_max)
                else:
                    mask |= cv2.inRange(img, r_min, r_max)

            canny = cv2.Canny(mask, 50, 100)
            cntrs, h = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for cntr in cntrs:
                rect = cv2.minAreaRect(cntr)
                width = rect[1][0]
                height = rect[1][1]
                if width > min_size and height > min_size:
                    boxes.append(FrameDetector.Box(color, cntr))

        return boxes

    def detect(self, show=False):
        img = self.camera.read()
        if img is None or self.camera.exited():
            return False

        bf_img = cv2.bilateralFilter(img, 1, 100, 200)
        hsv_img = cv2.cvtColor(bf_img, cv2.COLOR_BGR2HSV)

        boxes = self.getBoxes(hsv_img)
        if len(boxes) != 9:
            return True

        boxes.sort(key=lambda b: self.contourPrecedence(b, hsv_img.shape[1]))

        if show:
            self.displayBoxes(img, boxes)

        return True


if __name__ == '__main__':

    # TODO cmd line args
    src = '/dev/video0'
    prf = 'original'

    fd = FrameDetector(src, prf)

    while True:
        res = fd.detect(True)
        if not res:
            break

    # cv2.waitKey()
