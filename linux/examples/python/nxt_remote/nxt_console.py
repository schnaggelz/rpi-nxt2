#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import sys
import time
import signal

from nxt_remote_libs import periodic_timer as timer
from nxt_remote_libs import console_window as console

import nxt_remote_py as nxt


class RemoteConsole:

    VERSION = 1

    def __init__(self):
        self._window = console.ConsoleWindow()
        self._timer = None
        self._control = None
        self._stop = False
        self._counter = 0
        #self._window.print_at(2, 2, "Remote console V{:d}", RemoteConsole.VERSION)

    def periodic(self):
        self._window.print_at(8, 2, "Cycle {:6d}".format(self._counter))
        self._control.poll()
        self._window.print_at(10, 2, "M_A: {:6d}".format(self._control.motor_rcv(nxt.PORT_A, 0)))
        self._window.print_at(11, 2, "M_B: {:6d}".format(self._control.motor_rcv(nxt.PORT_B, 0)))
        self._window.print_at(12, 2, "M_C: {:6d}".format(self._control.motor_rcv(nxt.PORT_C, 0)))
        self._window.print_at(13, 2, "S_1: {:6d}".format(self._control.sensor_rcv(nxt.PORT_1, 0)))
        self._window.print_at(14, 2, "S_2: {:6d}".format(self._control.sensor_rcv(nxt.PORT_2, 0)))
        self._window.print_at(15, 2, "S_3: {:6d}".format(self._control.sensor_rcv(nxt.PORT_3, 0)))
        self._window.print_at(16, 2, "S_4: {:6d}".format(self._control.sensor_rcv(nxt.PORT_4, 0)))
        self._counter += 1

    def connect(self):
        self._window.print_status_at(4, 2, "Connecting to NXT...")
        self._control = nxt.Remote()
        if not self._control.connect():
            self._window.print_error_at(5, 2, "Could not connect!")
        return self._control.connected()

    def disconnect(self):
        if not self._control.disconnect():
            self._window.print_error_at(5, 2, "Could not disconnect!")

    def start(self):
        self._timer = timer.PeriodicTimer(0.1, self.periodic)
        self._stop = False
        while not self._stop:
            ch = self._window.get_char()
            self._window.put_char_at(20, 2, ch)

            if ch == ord('q'):
                break  # Exit the while loop

    def stop(self):
        self._stop = True
        self._timer.stop()


rc = RemoteConsole()


def handler(signum, frame):
    rc.stop()


signal.signal(signal.SIGINT, handler)

rc.connect()

rc.start()

# Application runs

rc.stop()

rc.disconnect()

time.sleep(1)

# if not rc.motor_fwd(nxt.PORT_A, 20):
#     logging.error('Could not control motor A!')

# if not rc.motor_stop(nxt.PORT_A):
#     logging.error('Could not control motor A!')
