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
import solver_machine as sm


class CubeSolver:
    VERSION = 1

    def __init__(self):
        self._window = console.ConsoleWindow()
        self._machine = sm.SolverMachine()
        self._timer = timer.PeriodicTimer(0.1, self.display)
        self._stop = False
        self._window.print_at(1, 1, "Remote console V{:d}".format(self.VERSION))

    def __del__(self):
        self.stop()

    def init(self):
        if self._machine.connect():
            self._window.print_status_at(5, 1, "Connected to NXT")
        else:
            self._window.print_error_at(5, 1, "Could not connect!")

    def exit(self):
        if self._machine.disconnect():
            self._window.print_status_at(5, 1, "Disconnected from NXT")
        else:
            self._window.print_error_at(5, 1, "Could not disconnect!")

    def start(self):
        self._machine.start()
        self._stop = False
        while not self._stop:
            time.sleep(0.5)
            ch = self._window.get_char()
            if ch == ord('h'):
                self._machine.turntable_home()
            if ch == ord('e'):
                self._machine.turntable_turn_ccw()
            if ch == ord('w'):
                self._machine.turntable_turn_cw()
            if ch == ord('z'):
                self._machine.turntable_home(True)
            if ch == ord('x'):
                self._machine.turntable_turn_ccw(True)
            if ch == ord('y'):
                self._machine.turntable_turn_cw(True)
            if ch == ord('o'):
                self._machine.grabber_home()
            if ch == ord('r'):
                self._machine.grabber_rest()
            if ch == ord('g'):
                self._machine.grabber_grab()
            if ch == ord('f'):
                self._machine.grabber_flip()
            if ch == ord('s'):
                self._machine.scanner_scan()
            if ch == ord('m'):
                self._machine.scanner_home()
            elif ch == ord('q'):
                break  # Exit the while loop

    def stop(self):
        self._window.print_status_at(5, 1, "STOP!")
        self._machine.stop()
        self._timer.stop()
        self._stop = True

    def display(self):
        self._window.print_at(10, 1, "MA: {:5d}".format(self._machine.decoder_values[0]))
        self._window.print_at(11, 1, "MB: {:5d}".format(self._machine.decoder_values[1]))
        self._window.print_at(12, 1, "MC: {:5d}".format(self._machine.decoder_values[2]))
        self._window.print_at(13, 1, "S1: {:5d}".format(self._machine.sensor_values[0]))
        self._window.print_at(14, 1, "S1: {:5d}".format(self._machine.sensor_values[1]))
        self._window.print_at(15, 1, "S1: {:5d}".format(self._machine.sensor_values[2]))
        self._window.print_at(16, 1, "S1: {:5d}".format(self._machine.sensor_values[3]))
        self._window.print_status_at(5, 1, "PC:{:6d}".format(self._machine.counter))
        self._window.refresh()


cbr = CubeSolver()


def handler(signum, frame):
    cbr.stop()


signal.signal(signal.SIGINT, handler)

cbr.init()

cbr.start()

time.sleep(1)

cbr.stop()

cbr.exit()
