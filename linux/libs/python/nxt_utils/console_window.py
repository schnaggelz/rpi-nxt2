#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).
import curses


class ConsoleWindow(object):
    def __init__(self):
        self._window = curses.initscr()
        curses.noecho()
        curses.cbreak()

    def __del__(self):
        curses.endwin()

    def get_char(self):
        return self._window.getch()

    def put_char_at(self, x, y, ch):
        self._window.addch(x, y, ch)
        self._window.refresh()

    def print_at(self, x, y, text):
        self._window.addstr(x, y, text)
        self._window.refresh()

    def print_status_at(self, x, y, text):
        self.print_at(x, y, "STATUS: {:20s}".format(text))

    def print_error_at(self, x, y, text):
        self.print_at(x, y, "ERROR: {:20s}".format(text))
