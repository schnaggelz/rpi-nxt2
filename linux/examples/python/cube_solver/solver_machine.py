#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).
import nxt_remote_py as nxt

from nxt_utils import periodic_timer as timer


class SolverMachine:
    VERSION = 1
    NXT = nxt.Remote()

    # Define motor ports
    MOTOR_TURN = NXT.PORT_A
    MOTOR_SCAN = NXT.PORT_B
    MOTOR_GRAB = NXT.PORT_C

    # Motor position constants
    MOTOR_GRAB_POSITION_HOME = 0
    MOTOR_GRAB_POSITION_REST = -35
    MOTOR_GRAB_POSITION_FLIP_PUSH = -90
    MOTOR_GRAB_POSITION_GRAB = -130
    MOTOR_GRAB_POSITION_FLIP = -240

    # Motor speed constants
    MOTOR_GRAB_SPEED_GRAB = 400
    MOTOR_GRAB_SPEED_FLIP = 600
    MOTOR_GRAB_SPEED_REST = 400

    def __init__(self):
        self._timer = None
        self._stop = True
        self._counter = 0

    def connect(self):
        return self.NXT.connect()

    def disconnect(self):
        return self.NXT.disconnect()

    def start(self):
        self._stop = False
        self._timer = timer.PeriodicTimer(0.01, self.periodic)

    def stop(self):
        self._stop = True
        self._timer.stop()

    def periodic(self):
        self.NXT.poll()
        self._counter += 1

    def run_to_pos(self, port, position, tolerance=3):
        pass
        # while ((encoder > (position + tolerance)) or (encoder < (position - tolerance))):
        #     time.sleep(0.01)
