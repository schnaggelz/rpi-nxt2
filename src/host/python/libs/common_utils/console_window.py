#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import curses

class ConsoleWindow(object):
    def __init__(self):
        self._window = curses.initscr()
        curses.noecho()
        curses.cbreak()

    def __del__(self):
        curses.endwin()

    def show_cursor(self):
        curses.echo()
        curses.curs_set(1)

    def hide_cursor(self):
        curses.curs_set(0)
        curses.noecho()

    def refresh(self):
        self._window.refresh()

    def get_string(self, y, x, len):
        return self._window.getstr(y, x, len)

    def get_char(self):
        return self._window.getch()

    def get_char_at(self, y, x):
        return self._window.getch(y, x)

    def put_char_at(self, y, x, ch):
        self._window.addch(y, x, ch)

    def print_at(self, y, x, str):
        self._window.addstr(y, x, str)

    def print_status_at(self, y, x, str):
        self.print_at(y, x, "STATUS: {:20s}".format(str))

    def print_error_at(self, y, x, str):
        self.print_at(y, x, "ERROR: {:20s}".format(str))

