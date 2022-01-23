#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import time
import numpy as np

from common_utils.console_window import ConsoleWindow


class SolverConsole:

    def __init__(self):
        self.__console_window = ConsoleWindow()
        self.__notation_offset = [6, 3]
        self.__values_offset = [4, 0]

    def print_version(self, name, version):
        self.__console_window.print_at(1, 1, "{} VERSION {:d}".format(name, version))

    def print_counter(self, value):
        self.__console_window.print_at(2, 1, "COUNTER: {:5d}".format(value))

    def print_fps(self, value):
        self.__console_window.print_at(2, 20, "FPS: {:2.1f}".format(value))

    def print_status(self, status):
        self.__console_window.print_status_at(3, 1, "STATUS: {}".format(status.ljust(30)))

    def print_values(self, values):
        row_offset = 1 + self.__values_offset[0]
        col_offset = 1 + self.__values_offset[1]

        for index, value in enumerate(values):
            self.__console_window.print_at(row_offset + index, col_offset,
                                           "V{}: {:5d}".format(index, value))

    def print_cube_notation(self, side, pattern):

        if side == 'U':
            row_offset = 1 + self.__notation_offset[0]
            col_offset = 14 + self.__notation_offset[1]
            trailer = '|'
        elif side == 'L':
            row_offset = 8 + self.__notation_offset[0]
            col_offset = 1 + self.__notation_offset[1]
            trailer = ''
        elif side == 'F':
            row_offset = 8 + self.__notation_offset[0]
            col_offset = 14 + self.__notation_offset[1]
            trailer = ''
        elif side == 'R':
            row_offset = 8 + self.__notation_offset[0]
            col_offset = 27 + self.__notation_offset[1]
            trailer = ''
        elif side == 'B':
            row_offset = 8 + self.__notation_offset[0]
            col_offset = 40 + self.__notation_offset[1]
            trailer = '|'
        elif side == 'D':
            row_offset = 15 + self.__notation_offset[0]
            col_offset = 14 + self.__notation_offset[1]
            trailer = '|'
        elif side == '?':
            row_offset = 1 + self.__notation_offset[0]
            col_offset = 50 + self.__notation_offset[1]
            trailer = '|'
        else:
            return None

        self.__console_window.print_at(row_offset + 0, col_offset,
                                       "|************{}".format(trailer))
        self.__console_window.print_at(row_offset + 1, col_offset,
                                       "|*{}1**{}2**{}3*{}".format(
                                           pattern[0], pattern[1], pattern[2], trailer))
        self.__console_window.print_at(row_offset + 2, col_offset,
                                       "|************{}".format(trailer))
        self.__console_window.print_at(row_offset + 3, col_offset,
                                       "|*{}4**{}5**{}6*{}".format(
                                           pattern[3], pattern[4], pattern[5], trailer))
        self.__console_window.print_at(row_offset + 4, col_offset,
                                       "|************{}".format(trailer))
        self.__console_window.print_at(row_offset + 5, col_offset,
                                       "|*{}7**{}8**{}9*{}".format(
                                           pattern[6], pattern[7], pattern[8], trailer))
        self.__console_window.print_at(row_offset + 6, col_offset,
                                       "|************{}".format(trailer))

    def get_char(self):
        return self.__console_window.get_char()

    def update(self):
        self.__console_window.refresh()


if __name__ == '__main__':
    display = SolverConsole()

    display.print_version("CUBER", 1)
    display.print_status("READY!")

    pattern = np.full(9, '?')

    pattern.fill('U')
    display.print_cube_notation('U', pattern)

    pattern.fill('L')
    display.print_cube_notation('L', pattern)

    pattern.fill('F')
    display.print_cube_notation('F', pattern)

    pattern.fill('R')
    display.print_cube_notation('R', pattern)

    pattern.fill('B')
    display.print_cube_notation('B', pattern)

    pattern.fill('D')
    display.print_cube_notation('D', pattern)

    pattern.fill('?')
    display.print_cube_notation('?', pattern)

    counter = 0
    display.print_counter(counter)

    v = np.zeros(8, dtype=int)
    display.print_values(v)

    display.update()

    while True:

        time.sleep(0.1)
        ch = display.get_char()

        if ch == ord('q'):
            break

        v += 1

        display.print_counter(counter)
        display.print_values(v)

        display.update()

        counter += 1
