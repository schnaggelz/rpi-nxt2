#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import time
import signal

from nxt_utils import console_window as console, periodic_timer as timer


class CubeSolver:
    VERSION = 1

    def __init__(self):
        self._window = console.ConsoleWindow()
        self._timer = None
        self._stop = False
        self._counter = 0
        self._window.print_at(1, 1, "Remote console V{:d}".format(self.VERSION))

    def periodic(self):
        self._window.print_at(8, 1, "Cycle {:5d}".format(self._counter))
        self.NXT.poll()
        self._counter += 1

    def connect(self):
        self._window.print_status_at(4, 1, "Connecting to NXT...")
        if not self.NXT.connect():
            self._window.print_error_at(5, 1, "Could not connect!")
        return self.NXT.connected()

    def disconnect(self):
        if not self.NXT.disconnect():
            self._window.print_error_at(5, 1, "Could not disconnect!")

    def start(self):
        self._timer = timer.PeriodicTimer(0.1, self.periodic)
        self._stop = False
        while not self._stop:
            ch = self._window.get_char()
            if ch == ord('q'):
                break  # Exit the while loop

    def stop(self):
        self._stop = True
        self._timer.stop()


cbr = CubeSolver()


def handler(signum, frame):
    cbr.stop()


signal.signal(signal.SIGINT, handler)

cbr.connect()

cbr.start()

# Application runs

cbr.stop()

cbr.disconnect()

time.sleep(1)
