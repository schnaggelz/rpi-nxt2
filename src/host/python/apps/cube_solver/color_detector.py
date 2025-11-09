#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import cv2
import numpy as np
import itertools
import signal
import dataclasses

from cube_colors import CubeColors

from vision_utils.pi_camera import Camera
from vision_utils.fps_calcularor import FpsCalculator
from vision_utils.video_sender import VideoSender


class ColorDetector:

    DEBUG_MODE = False

    @dataclasses.dataclass
    class Parameters:
        size_threshold_min: int = 50
        size_threshold_max: int = 100
        precedence_tolerance: int = 5
        white_sensitivity: int = 75
        num_boxes: int = 9
        square_tolerance: float = 0.2
        font_color: tuple = (255, 255, 255)

    PARAMS = Parameters()

    class Box:
        __slots__ = ['color', 'contour']

        def __init__(self, color, contour):
            self.color = color
            self.contour = contour

    def __init__(self, output=None, profile='original'):
        self._colors = CubeColors(profile).colors
        self._video_sink = None

        self._output = output
        self._profile = profile
        self._fps = FpsCalculator()
        self._camera = Camera(lores_size=(320, 240),
                              lores_callback=self.analyze)
        self._camera.open()

    @property
    def video_sink(self):
        return self._video_sink

    @video_sink.setter
    def video_sink(self, sink):
        self._video_sink = sink

    @staticmethod
    def get_hsv(img):
        rgb = cv2.cvtColor(img, cv2.COLOR_YUV420p2RGB)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

        return hsv

    @staticmethod
    def contour_precedence(box, cols):
        tolerance = 5
        origin = cv2.boundingRect(box.contour)
        return (((origin[1] + origin[3]) / 2 // tolerance) * tolerance) * cols + (origin[0] + origin[2]) / 2

    @staticmethod
    def get_color_contours(img_hsv, ranges):
        mask = np.zeros(img_hsv.shape, dtype=np.uint8)
        for r in ranges:
            lower = np.array(r[0], np.uint8)
            upper = np.array(r[1], np.uint8)
            color_mask = cv2.inRange(img_hsv, lower, upper)
            color_mask = cv2.merge([color_mask, color_mask, color_mask])
            mask = cv2.bitwise_or(mask, color_mask)

        canny = cv2.Canny(mask, 0, 50)
        cntrs, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        return cntrs

    @staticmethod
    def get_white_contours(img_hsv):
        sensitivity = ColorDetector.PARAMS.white_sensitivity
        lower_white = np.array([0, 0, 255 - sensitivity])
        upper_white = np.array([255, sensitivity, 255])

        mask = cv2.inRange(img_hsv, lower_white, upper_white)

        canny = cv2.Canny(mask, 0, 50)
        cntrs, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        return cntrs

    @staticmethod
    def filter_contours(cntrs, min_size, max_size):
        filtered_cntrs = []
        for cntr in cntrs:
            rect = cv2.minAreaRect(cntr)
            width = rect[1][0]
            height = rect[1][1]
            lower_tolerance = 1 - ColorDetector.PARAMS.square_tolerance
            upper_tolerance = 1 + ColorDetector.PARAMS.square_tolerance
            if width > min_size and height > min_size:
                if width < max_size and height < max_size:
                    if lower_tolerance <= (width / height) <= upper_tolerance:
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
            text = "#{}:{}".format(idx, box.color.location)
            img = cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                              0.5, ColorDetector.PARAMS.font_color, 1, cv2.LINE_AA)
        return img

    def display_rate(self, img):
        img = cv2.putText(img, "FPS:{:2.1f}".format(self._fps()), (5, 15), cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, ColorDetector.PARAMS.font_color, 1, cv2.LINE_AA)
        return img

    def get_boxes(self, img):
        img_hsv = self.get_hsv(img)
        boxes = []
        for color in self._colors:
            cntrs = self.get_color_contours(img_hsv, color.ranges)
            cntrs = self.filter_contours(cntrs,
                                         self.PARAMS.size_threshold_min,
                                         self.PARAMS.size_threshold_max)
            for cntr in cntrs:
                boxes.append(self.Box(color, cntr))

        white_cntrs = self.get_white_contours(img_hsv)
        white_cntrs = self.filter_contours(white_cntrs,
                                           self.PARAMS.size_threshold_min,
                                           self.PARAMS.size_threshold_max)
        for white_cntr in white_cntrs:
            boxes.append(self.Box(CubeColors.WHITE, white_cntr))

        return boxes

    def analyze(self, img):

        if img is None:
            return False

        boxes = self.get_boxes(img)
        num_boxes = len(boxes)

        if num_boxes == 9 or (self.DEBUG_MODE and num_boxes > 0):
            (boxes, _) = self.sort_boxes(boxes, True)
            pattern = np.full(9, '?')
            sorted_rows = []
            unsorted_row = []
            row_counter = 0
            for (i, box) in enumerate(boxes, 1):
                unsorted_row.append(box)
                if i % 3 == 0:
                    (sorted_row, _) = self.sort_boxes(unsorted_row, False)
                    sorted_rows.append(sorted_row)
                    for (j, sorted_box) in enumerate(sorted_row):
                        pattern[3 * row_counter + j] = sorted_box.color.location

                    unsorted_row = []
                    row_counter += 1

            if self._output:
                if not self._output.full():
                    self._output.put(pattern)

            if self._video_sink:
                img = self.display_boxes(img, list(itertools.chain(*sorted_rows)))

        if self._video_sink:
            self.display_rate(img)
            self._video_sink.send(img)

    def start(self):
        self._camera.start()

    def stop(self):
        self._camera.stop()


if __name__ == '__main__':
    sender = VideoSender('192.168.242.163', 4243)
    sender.connect()

    detector = ColorDetector()
    detector.video_sink = sender
    detector.start()


    def handler(signum, frame):
        detector.stop()


    signal.signal(signal.SIGINT, handler)
