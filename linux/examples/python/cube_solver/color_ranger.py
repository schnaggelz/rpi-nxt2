#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import cv2
import numpy as np

from vision_utils.video_receiver import VideoReceiver


class ColorRanger:

    def __init__(self, sink=None):
        self._sink = sink

        cv2.namedWindow("Trackbars")

        def nothing(x):
            pass

        cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
        cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
        cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

    def range(self, img):

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])

        mask = cv2.inRange(hsv, lower_range, upper_range)
        res = cv2.bitwise_and(img, img, mask=mask)
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        stacked = np.hstack((mask_3, img, res))

        cv2.imshow('Trackbars', cv2.resize(stacked, None, fx=0.4, fy=0.4))


if __name__ == '__main__':

    ranger = ColorRanger()

    receiver = VideoReceiver(4243)
    receiver.connect()

    while True:
        try:
            img = receiver.receive()
            ranger.range(img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        except KeyboardInterrupt:
            pass
        finally:
            pass #TODO
