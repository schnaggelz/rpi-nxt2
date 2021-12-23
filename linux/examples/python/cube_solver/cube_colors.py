from collections import defaultdict

import cv2
import numpy as np

import camera_source as cam


class CubeColors:
    PROFILE_COLORS = (
        ('red', [1], ([0, 50, 50], [5, 255, 255])),
        ('orange', [1], ([6, 50, 100], [20, 255, 255])),
        ('yellow', [1], ([21, 50, 100], [40, 255, 255])),
        # ('citron', [], ([31, 50, 100], [50, 255, 255])),
        ('green', [1], ([41, 50, 50], [70, 255, 255])),
        # ('turquoise', [], ([71, 50, 50], [75, 255, 255])),
        ('cyan', [], ([71, 50, 100], [85, 255, 255])),
        # ('ocean', [], ([86, 50, 50], [100, 255, 255])),
        ('blue', [1], ([86, 50, 50], [120, 255, 255])),
        # ('violet', [], ([121, 10, 50], [135, 255, 255])),
        ('magenta', [], ([136, 10, 50], [170, 255, 255])),
        # ('raspberry', [], ([151, 50, 50], [170, 255, 255])),
        ('red', [1], ([171, 50, 50], [179, 255, 255])),
        ('white', [1], ([0, 0, 200], [179, 50, 255]))
    )

    PROFILES = {
        'original': 1,
        'niko': 2
    }

    def __init__(self, profile):
        self.colors = CubeColors.ranges(profile)

    def calibrate(self):
        # TODO ;-)
        pass

    @staticmethod
    def range(color):
        r = []
        for entry in CubeColors.PROFILE_COLORS:
            if entry[0] == color:
                r.append(entry[1])
        return r

    @staticmethod
    def ranges(profile):
        pix = CubeColors.PROFILES[profile]
        results = defaultdict(list)
        for entry in CubeColors.PROFILE_COLORS:
            color = entry[0]
            switch = entry[1]
            if pix in switch:
                results[color].append(entry[2])
        return results

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
