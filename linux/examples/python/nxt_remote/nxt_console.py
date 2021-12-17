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

import nxt_remote_py as nxt


class RemoteConsole:
    VERSION = 1

    def __init__(self):
        self._window = console.ConsoleWindow()
        self._timer = None
        self._control = None
        self._stop = False
        self._counter = 0
        self._window.print_at(1, 1, "Remote console V{:d}".format(RemoteConsole.VERSION))

    def periodic(self):
        self._window.print_at(8, 1, "Cycle {:5d}".format(self._counter))
        self._control.poll()
        self.display()
        self._counter += 1

    def connect(self):
        self._window.print_status_at(4, 1, "Connecting to NXT...")
        self._control = nxt.Remote()
        if not self._control.connect():
            self._window.print_error_at(5, 1, "Could not connect!")
        return self._control.connected()

    def disconnect(self):
        if not self._control.disconnect():
            self._window.print_error_at(5, 1, "Could not disconnect!")

    def start(self):
        self._timer = timer.PeriodicTimer(0.1, self.periodic)
        self._stop = False
        while not self._stop:
            ch = self._window.get_char()
            self._window.put_char_at(20, 2, ch)

            if ch == ord('q'):
                break  # Exit the while loop
            elif ch == ord('f'):
                ch = self._window.get_char()
                if ch == ord('a'):
                    break

    def stop(self):
        self._stop = True
        self._timer.stop()

    def display(self):
        self._window.print_at(10, 1, "M_A(0): {:5d}".format(self._control.motor_rcv(nxt.PORT_A, 0)))
        self._window.print_at(11, 1, "M_B(0): {:5d}".format(self._control.motor_rcv(nxt.PORT_B, 0)))
        self._window.print_at(12, 1, "M_C(0): {:5d}".format(self._control.motor_rcv(nxt.PORT_C, 0)))
        self._window.print_at(10, 21, "M_A(1): {:5d}".format(self._control.motor_rcv(nxt.PORT_A, 1)))
        self._window.print_at(11, 21, "M_B(1): {:5d}".format(self._control.motor_rcv(nxt.PORT_B, 1)))
        self._window.print_at(12, 21, "M_C(1): {:5d}".format(self._control.motor_rcv(nxt.PORT_C, 1)))
        self._window.print_at(13, 1, "S_1(0): {:5d}".format(self._control.sensor_rcv(nxt.PORT_1, 0)))
        self._window.print_at(14, 1, "S_2(0): {:5d}".format(self._control.sensor_rcv(nxt.PORT_2, 0)))
        self._window.print_at(15, 1, "S_3(0): {:5d}".format(self._control.sensor_rcv(nxt.PORT_3, 0)))
        self._window.print_at(16, 1, "S_4(0): {:5d}".format(self._control.sensor_rcv(nxt.PORT_4, 0)))


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
