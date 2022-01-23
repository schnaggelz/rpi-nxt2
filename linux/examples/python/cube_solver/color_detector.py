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

from cube_colors import CubeColors

from vision_utils.pi_camera import Camera
from vision_utils.fps_calcularor import FpsCalculator
from vision_utils.video_sender import VideoSender


class ColorDetector:
    class __Box:
        __slots__ = ['color', 'contour']

        def __init__(self, color, contour):
            self.color = color
            self.contour = contour

    def __init__(self, output, profile='original'):
        self.__colors = CubeColors(profile).colors
        self.__video_sink = None

        self.__output = output
        self.__profile = profile
        self.__size_threshold = 25
        self.__fps = FpsCalculator()
        self.__camera = Camera(width=320,
                               height=320,
                               framerate=20,
                               callback=self.analyze)
        self.__camera.open()

    @property
    def video_sink(self):
        return self.__video_sink

    @video_sink.setter
    def video_sink(self, sink):
        self.__video_sink = sink

    @staticmethod
    def get_hsv(img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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
        sensitivity = 75
        lower_white = np.array([0, 0, 255 - sensitivity])
        upper_white = np.array([255, sensitivity, 255])

        mask = cv2.inRange(img_hsv, lower_white, upper_white)

        canny = cv2.Canny(mask, 0, 50)
        cntrs, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

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
    def display_boxes(img, boxes):
        for idx, box in enumerate(boxes):
            x, y, w, h = cv2.boundingRect(box.contour)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            text = "#{}:{}".format(idx, box.color.location)
            img = cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                              0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return img

    def display_rate(self, img):
        img = cv2.putText(img, "FPS:{:2.1f}".format(self.__fps()), (5, 15), cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return img

    def get_boxes(self, img):
        img_hsv = self.get_hsv(img)
        boxes = []
        for color in self.__colors:
            cntrs = self.get_color_contours(img_hsv, color.ranges)
            cntrs = self.filter_contours(cntrs, self.__size_threshold)
            for cntr in cntrs:
                boxes.append(self.__Box(color, cntr))

        white_cntrs = self.get_white_contours(img_hsv)
        white_cntrs = self.filter_contours(white_cntrs, self.__size_threshold)
        for white_cntr in white_cntrs:
            boxes.append(self.__Box(CubeColors.WHITE, white_cntr))

        return boxes

    def analyze(self, img):

        if img is None:
            return False

        boxes = self.get_boxes(img)

        if len(boxes) == 9:
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

            if not self.__output.full():
                self.__output.put(pattern)

            img = self.display_boxes(img, list(itertools.chain(*sorted_rows)))

        if self.__video_sink is not None:
            self.display_rate(img)
            self.__video_sink.send(img)

    def start(self):
        self.__camera.start()

    def stop(self):
        self.__camera.stop()


if __name__ == '__main__':
    sender = VideoSender('192.168.242.137', 4243)
    sender.connect()

    detector = ColorDetector()
    detector.video_sink = sender
    detector.start()


    def handler(signum, frame):
        detector.stop()


    signal.signal(signal.SIGINT, handler)
