from collections import defaultdict

import cv2
import numpy as np

import camera_source as cam


class CubeColors:
    PROFILE_COLORS = (
        ('red', [1, 2], ([0, 100, 150], [10, 255, 255])),
        ('orange', [2], ([11, 100, 150], [20, 255, 255])),
        ('yellow', [1, 2], ([21, 100, 150], [35, 255, 255])),
        ('citron', [2], ([36, 100, 150], [50, 255, 255])),
        ('green', [1], ([51, 100, 100], [75, 255, 255])),
        ('cyan', [2], ([76, 100, 100], [95, 255, 255])),
        # ('ocean', [], ([96, 100, 100], [100, 255, 255])),
        ('blue', [1], ([96, 100, 100], [120, 255, 255])),
        ('violet', [1], ([121, 50, 50], [145, 255, 255])),
        ('magenta', [1], ([146, 100, 50], [165, 255, 255])),
        # ('raspberry', [], ([166, 100, 50], [170, 255, 255])),
        ('red', [1, 2], ([171, 150, 150], [180, 255, 255])),
        ('white', [2], ([0, 0, 200], [179, 55, 255]))
    )

    PROFILES = {
        'original': 1,
        'niko': 2
    }

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
