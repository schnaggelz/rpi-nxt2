#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).
import time

import nxt_remote_py as nxt_api
from nxt_utils import periodic_timer as timer


class SolverMachine:
    VERSION = 1

    # Define motor ports
    MOTOR_TURN = nxt_api.PORT_A
    MOTOR_SCAN = nxt_api.PORT_B
    MOTOR_GRAB = nxt_api.PORT_C

    # Motor position constants
    MOTOR_GRAB_POSITION_HOME = 0
    MOTOR_GRAB_POSITION_REST = -35
    MOTOR_GRAB_POSITION_FLIP_PUSH = -90
    MOTOR_GRAB_POSITION_GRAB = -130
    MOTOR_GRAB_POSITION_FLIP = -240

    # Motor speed constants
    MOTOR_GRAB_SPEED_GRAB = 40
    MOTOR_GRAB_SPEED_FLIP = 60
    MOTOR_GRAB_SPEED_REST = 40

    MOTOR_TURN_SPEED = 50

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

    def run_to_pos(self, port, position, tolerance=3):
        cur_pos = self._nxt_rc.motor_rcv(port, 0)
        if cur_pos < position - tolerance:
            self._nxt_rc.motor_cmd(port, +self.MOTOR_TURN_SPEED, position)
            # while cur_pos < position - tolerance:
            #     time.sleep(0.001)
            #     cur_pos = self._nxt_rc.motor_rcv(port, 0)
        elif cur_pos > position + tolerance:
            self._nxt_rc.motor_cmd(port, self.MOTOR_TURN_SPEED, position)
            # while cur_pos > position + tolerance:
            #     time.sleep(0.001)
            #     cur_pos = self._nxt_rc.motor_rcv(port, 0)
        # self._nxt_rc.motor_stop(port)

    def turntable_forward(self):
        self.run_to_pos(self.MOTOR_TURN, 250)

    def turntable_reverse(self):
        self.run_to_pos(self.MOTOR_TURN, -250)


if __name__ == '__main__':
    sm = SolverMachine()
