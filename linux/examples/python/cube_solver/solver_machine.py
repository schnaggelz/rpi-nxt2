#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import time
from enum import Enum

import nxt_remote_py as nxt_api
from nxt_utils import periodic_timer as timer


class SolverMachine:
    VERSION = 1

    # Motor speed constants
    MOTOR_GRAB_SPEED = 50
    MOTOR_TURN_SPEED = 100
    MOTOR_SCAN_SPEED = 50

    MOTOR_TURN = nxt_api.PORT_A
    MOTOR_SCAN = nxt_api.PORT_B
    MOTOR_GRAB = nxt_api.PORT_C

    class TurntablePosition(Enum):
        HOME = 1
        CW = -270
        CCW = 270

    class GrabberPosition(Enum):
        HOME = 1
        GRAB = 170
        FLIP = 60
        REST = 220

    class ScannerPosition(Enum):
        HOME = 1
        SCAN = 30

    def __init__(self):
        self._timer = None
        self._counter = 0
        self._decoder_values = [0, 0, 0]
        self._sensor_values = [0, 0, 0, 0]
        self._nxt_rc = nxt_api.Remote()

    @property
    def decoder_values(self):
        return self._decoder_values

    @property
    def sensor_values(self):
        return self._sensor_values

    @property
    def counter(self):
        return self._counter

    def connect(self):
        return self._nxt_rc.connect()

    def disconnect(self):
        return self._nxt_rc.disconnect()

    def start(self):
        self._timer = timer.PeriodicTimer(0.001, self.periodic)

    def stop(self):
        self._timer.stop()
        self._nxt_rc.motor_stop(self.MOTOR_TURN)
        self._nxt_rc.motor_stop(self.MOTOR_GRAB)
        self._nxt_rc.motor_stop(self.MOTOR_SCAN)

    def periodic(self):
        self._nxt_rc.poll()
        self._decoder_values[0] = self._nxt_rc.motor_rcv(nxt_api.PORT_A, 0)
        self._decoder_values[1] = self._nxt_rc.motor_rcv(nxt_api.PORT_B, 0)
        self._decoder_values[2] = self._nxt_rc.motor_rcv(nxt_api.PORT_C, 0)
        self._sensor_values[0] = self._nxt_rc.sensor_rcv(nxt_api.PORT_1, 0)
        self._sensor_values[1] = self._nxt_rc.sensor_rcv(nxt_api.PORT_2, 0)
        self._sensor_values[2] = self._nxt_rc.sensor_rcv(nxt_api.PORT_3, 0)
        self._sensor_values[3] = self._nxt_rc.sensor_rcv(nxt_api.PORT_4, 0)
        self._counter += 1

    def run_to_pos(self, port, speed, position, tolerance):
        cur_pos = self._nxt_rc.motor_rcv(port, 0)
        if cur_pos < position - tolerance:
            self._nxt_rc.motor_cmd(port, speed, position, tolerance)
            while cur_pos < position - tolerance:
                time.sleep(0.001)
                cur_pos = self._nxt_rc.motor_rcv(port, 0)
        elif cur_pos > position + tolerance:
            self._nxt_rc.motor_cmd(port, -speed, position, tolerance)
            while cur_pos > position + tolerance:
                time.sleep(0.001)
                cur_pos = self._nxt_rc.motor_rcv(port, 0)
        # self._nxt_rc.motor_stop(port)

    def turntable_turn_ccw(self, load=False):
        tolerance = 10 if load else 50
        self.run_to_pos(self.MOTOR_TURN, self.MOTOR_TURN_SPEED, self.TurntablePosition.CCW.value, tolerance)

    def turntable_turn_cw(self, load=False):
        tolerance = 10 if load else 50
        self.run_to_pos(self.MOTOR_TURN, self.MOTOR_TURN_SPEED, self.TurntablePosition.CW.value, tolerance)

    def turntable_home(self, load=False):
        tolerance = 10 if load else 50
        self.run_to_pos(self.MOTOR_TURN, self.MOTOR_TURN_SPEED, self.TurntablePosition.HOME.value, tolerance)

    def grabber_grab(self):
        tolerance = 10
        self.run_to_pos(self.MOTOR_GRAB, self.MOTOR_GRAB_SPEED, self.GrabberPosition.GRAB.value, tolerance)

    def grabber_flip(self):
        tolerance = 10
        self.run_to_pos(self.MOTOR_GRAB, self.MOTOR_GRAB_SPEED, self.GrabberPosition.FLIP.value, tolerance)

    def grabber_rest(self):
        tolerance = 10
        self.run_to_pos(self.MOTOR_GRAB, self.MOTOR_GRAB_SPEED, self.GrabberPosition.REST.value, tolerance)

    def grabber_home(self):
        tolerance = 10
        self.run_to_pos(self.MOTOR_GRAB, self.MOTOR_GRAB_SPEED, self.GrabberPosition.HOME.value, tolerance)

    def scanner_scan(self):
        tolerance = 10
        self.run_to_pos(self.MOTOR_SCAN, self.MOTOR_SCAN_SPEED, self.ScannerPosition.SCAN.value, tolerance)

    def scanner_home(self):
        tolerance = 10
        self.run_to_pos(self.MOTOR_SCAN, self.MOTOR_SCAN_SPEED, self.ScannerPosition.HOME.value, tolerance)


if __name__ == '__main__':
    sm = SolverMachine()
