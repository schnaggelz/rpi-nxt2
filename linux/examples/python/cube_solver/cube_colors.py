import cv2
import numpy as np

import camera_source as cam


class CubeColors:
    COLORS = (
        # ('red', ([0, 150, 150], [5, 255, 255])),
        # ('orange', ([6, 200, 200], [15, 255, 255])),
        # ('yellow', ([21, 200, 200], [30, 255, 255])),
        # ('green', ([31, 150, 155], [75, 255, 255])),
        # ('blue', ([76, 100, 100], [135, 255, 255])),
        ('purple', ([136, 165, 100], [135, 255, 255])),
        # ('red', ([166, 150, 150], [180, 255, 255])),
        ('white', ([0, 0, 200], [179, 55, 255]))
    )

    @staticmethod
    def range(color):
        r = []
        for entry in CubeColors.COLORS:
            if entry[0] == color:
                r.append(entry[1])
        return r

    @staticmethod
    def ranges():
        r = {}
        for entry in CubeColors.COLORS:
            color = entry[0]
            if color not in r:
                r[color] = []
            r[color].append(entry[1])
        return r

    @staticmethod
    def print():
        for color, ranges in CubeColors.ranges().items():
            for range in ranges:
                r_min = np.array(range[0], np.uint8)
                r_max = np.array(range[1], np.uint8)
                print("{}:{}.{}.{}:{}.{}.{}".format(color,
                                                    r_min[0], r_min[1], r_min[2],
                                                    r_max[0], r_max[1], r_max[2]))


if __name__ == '__main__':
    CubeColors.print()
