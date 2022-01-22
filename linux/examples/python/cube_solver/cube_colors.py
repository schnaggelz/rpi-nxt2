from typing import NamedTuple

import numpy as np


class CubeColors:
    class Color(NamedTuple):
        name: str
        location: str
        profile_id: int
        ranges: tuple

    PROFILE_COLORS = [
        Color('red', 'U', 1, [[[0, 100, 50], [5, 255, 255]], [[166, 100, 50], [179, 255, 255]]]),
        Color('orange', 'D', 1, [[[3, 100, 100], [17, 255, 255]]]),
        Color('yellow', 'R', 1, [[[17, 50, 100], [40, 255, 255]]]),
        Color('citron', 'X', 0, [[[31, 50, 100], [50, 255, 255]]]),
        Color('green', 'F', 1, [[[41, 50, 50], [70, 255, 255]]]),
        Color('turquoise', 'X', 0, [[[71, 50, 50], [75, 255, 255]]]),
        Color('cyan', 'X', 0, [[[71, 50, 100], [85, 255, 255]]]),
        Color('ocean', 'X', 0, [[[86, 50, 50], [100, 255, 255]]]),
        Color('blue', 'B', 1, [[[86, 100, 50], [120, 255, 255]]]),
        Color('violet', 'X', 0, [[[121, 10, 50], [135, 255, 255]]]),
        Color('magenta', 'X', 0, [[[136, 10, 50], [150, 255, 255]]]),
        Color('raspberry', 'X', 0, [[[151, 50, 50], [165, 255, 255]]]),
    ]

    WHITE = Color('white', 'L', 1, None)

    PROFILES = {
        'original': 1,
        'niko': 2
    }

    def __init__(self, profile):
        self.__colors = CubeColors.colors_of_profile(profile)

    @property
    def colors(self):
        return self.__colors

    def calibrate(self):
        # TODO ;-)
        pass

    def ranges(self, color_name):
        colors = []
        for color in self.__colors:
            if color.name == color_name:
                colors.append(color)
        return colors

    @staticmethod
    def colors_of_profile(profile):
        profile_id = CubeColors.PROFILES[profile]
        colors = []
        for color in CubeColors.PROFILE_COLORS:
            if color.profile_id == profile_id:
                colors.append(color)
        return colors

    def print(self):
        for color in self.__colors:
            for rng in color.ranges:
                r_min = np.array(rng[0], np.uint8)
                r_max = np.array(rng[1], np.uint8)
                print("color:{}; location:{}; profile:{}; "
                      "range:{},{},{}:{},{},{}".format(color.name,
                                                       color.location, color.profile_id,
                                                       r_min[0], r_min[1], r_min[2],
                                                       r_max[0], r_max[1], r_max[2]))


if __name__ == '__main__':
    cc = CubeColors('original')
    cc.print()
