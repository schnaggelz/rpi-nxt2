#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import cv2
import numpy as np

class CvVersion:
    @staticmethod
    def version():
        (major, minor, _) = cv2.__version__.split(".")
        return major, minor

    @staticmethod
    def print():
        major, minor = CvVersion.version()
        print("OpenCV version: {}.{}".format(major, minor))