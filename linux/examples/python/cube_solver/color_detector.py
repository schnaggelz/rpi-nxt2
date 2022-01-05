#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import time
import cv2
import numpy as np
import itertools

from cube_colors import CubeColors
from cam_utils.pi_camera import Camera


class ColorDetector:
    class Box:
        __slots__ = ['color', 'contour']

        def __init__(self, color, contour):
            self.color = color
            self.contour = contour

    def __init__(self, profile='original', sink=None):
        self._sink = sink
        self._profile = profile
        self._size_threshold = 50
        self._camera = Camera(640, 480)
        self._camera.open()

    @staticmethod
    def contour_precedence(box, cols):
        tolerance = 10
        origin = cv2.boundingRect(box.contour)
        return (((origin[1] + origin[3]) / 2 // tolerance) * tolerance) * cols + (origin[0] + origin[2]) / 2

    @staticmethod
    def get_contours(img_hsv, ranges):
        mask = None
        for r in ranges:
            lower = np.array(r[0], np.uint8)
            upper = np.array(r[1], np.uint8)
            if mask is None:
                mask = cv2.inRange(img_hsv, lower, upper)
            else:
                mask += cv2.inRange(img_hsv, lower, upper)

        canny = cv2.Canny(mask, 50, 100)
        cntrs, h = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        return cntrs

    @staticmethod
    def get_contours_2(img_hsv, ranges):
        mask = np.zeros(img_hsv.shape, dtype=np.uint8)

        morph_size = 3
        elem_o = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_size + 1, morph_size + 1))
        elem_c = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_size - 1, morph_size - 1))

        cntrs = []

        for r in ranges:
            lower = np.array(r[0], np.uint8)
            upper = np.array(r[1], np.uint8)
            color_mask = cv2.inRange(img_hsv, lower, upper)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, elem_o, iterations=1)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, elem_c, iterations=5)

            color_mask = cv2.merge([color_mask, color_mask, color_mask])
            mask |= color_mask

            gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            c, h = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cntrs.extend(c)

        return cntrs

    @staticmethod
    def filter_contours(cntrs, min_size):
        filtered_cntrs = []
        for cntr in cntrs:
            rect = cv2.minAreaRect(cntr)
            width = rect[1][0]
            height = rect[1][1]
            if width > min_size and height > min_size:
                if 0.8 <= (width / height) <= 1.2:
                    filtered_cntrs.append(cntr)
        return filtered_cntrs

    @staticmethod
    def sort_boxes(boxes, vertical=False):
        i = 0
        if vertical:
            i = 1

        rects = [cv2.boundingRect(box.contour) for box in boxes]
        boxes, rects = zip(*sorted(zip(boxes, rects),
                                   key=lambda b: b[1][i], reverse=False))
        return boxes, rects

    @staticmethod
    def display_boxes(img, boxes):
        for idx, box in enumerate(boxes):
            x, y, w, h = cv2.boundingRect(box.contour)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            text = "#{}:{}".format(idx, box.color)
            img = cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                              0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return img

    @staticmethod
    def display_rate(img, start_time):
        elapsed_time = time.time() - start_time
        secs_elapsed = elapsed_time % 60
        fps = 1 / secs_elapsed
        img = cv2.putText(img, "FPS:{}".format(fps), (5, 15), cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return img

    def get_boxes(self, img):
        img_filtered = cv2.bilateralFilter(img, 1, 100, 200)
        img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
        boxes = []
        for color, ranges in cc.CubeColors.ranges(self._profile).items():
            cntrs = self.get_contours_2(img_hsv, ranges)
            cntrs = self.filter_contours(cntrs, self._size_threshold)
            for cntr in cntrs:
                boxes.append(self.Box(color, cntr))

        return boxes

    def detect(self, show=False):
        start_time = time.time()

        img = self._camera.read()
        if img is None or self._camera.exited():
            return False

        boxes = self.get_boxes(img)

        if len(boxes) == 9:
            (boxes, _) = self.sort_boxes(boxes, True)
            rows = []
            row = []
            for (i, box) in enumerate(boxes, 1):
                row.append(box)
                if i % 3 == 0:
                    (boxes, _) = self.sort_boxes(row, False)
                    rows.append(boxes)
                    row = []

            if show:
                img = self.display_boxes(img, list(itertools.chain(*rows)))

        if show:
            img = self.display_rate(img, start_time)
            cv2.imshow("detector", img)

        return True


if __name__ == '__main__':

    # TODO cmd line args
    src = '/dev/video0'
    prf = 'original'

    fd = ColorDetector(src, prf)

    while True:
        res = fd.detect(True)
        if not res:
            break

    # cv2.waitKey()
