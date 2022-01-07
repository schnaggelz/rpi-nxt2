#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import time
import cv2
import numpy as np
import itertools

from cube_colors import CubeColors
from vision_utils.pi_camera import Camera


class ColorDetector:
    class Box:
        __slots__ = ['color', 'contour']

        def __init__(self, color, contour):
            self.color = color
            self.contour = contour

    def __init__(self, profile='original', sink=None):
        self._sink = sink
        self._profile = profile
        self._size_threshold = 30
        self._time = time.time()
        self._camera = Camera(width=320,
                              height=320,
                              framerate=20,
                              callback=self.analyze)
        self._camera.open()

    @staticmethod
    def get_hsv(img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # set all hue values >160 to 0 (red color)
        h, s, v = cv2.split(hsv)
        h_mask = cv2.inRange(h, 0, 160)
        h = cv2.bitwise_and(h, h, mask=h_mask)
        hsv = cv2.merge((h, s, v)).astype(float)

        return hsv

    @staticmethod
    def contour_precedence(box, cols):
        tolerance = 5
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

        canny = cv2.Canny(mask, 50, 50)
        _, cntrs, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        return cntrs

    @staticmethod
    def filter_contours(cntrs, min_size):
        filtered_cntrs = []
        for cntr in cntrs:
            rect = cv2.minAreaRect(cntr)
            width = rect[1][0]
            height = rect[1][1]
            if width > min_size and height > min_size:
                if 0.9 <= (width / height) <= 1.1:
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
    def overlay_boxes(img, boxes):
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
        img = cv2.putText(img, "FPS:{:2.1f}".format(fps), (5, 15), cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return img

    def get_boxes(self, img):
        img_hsv = self.get_hsv(img)
        boxes = []
        for color, ranges in CubeColors.ranges(self._profile).items():
            cntrs = self.get_contours(img_hsv, ranges)
            cntrs = self.filter_contours(cntrs, self._size_threshold)
            for cntr in cntrs:
                boxes.append(self.Box(color, cntr))

        return boxes

    def analyze(self, img):

        if img is None:
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
            img = self.overlay_boxes(img, list(itertools.chain(*rows)))

        if self._sink is not None:
            self.display_rate(img, self._time)

        if self._sink is not None:
            self._sink.send(img)

        self._time = time.time()

    def start(self):
        self._camera.run()


if __name__ == '__main__':

    from vision_utils.video_sender import VideoSender

    sender = VideoSender('treich-dt-1', 1234)
    sender.connect()

    detector = ColorDetector(sink=sender)
    detector.start()

