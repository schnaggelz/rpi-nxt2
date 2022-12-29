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
import nxt_remote_py as nxt

from enum import Enum

from common_utils.periodic_timer import PeriodicTimer


class SolverMachine:
    VERSION = 1

    REST_TIME = 0.2

    # Motor speed constants
    MOTOR_GRAB_SPEED = 50
    MOTOR_TURN_SPEED = 100
    MOTOR_SCAN_SPEED = 50

    MOTOR_TURN = nxt.PORT_A
    MOTOR_SCAN = nxt.PORT_B
    MOTOR_GRAB = nxt.PORT_C

    class Face(Enum):
        UP = 0
        FRONT = 1
        RIGHT = 2
        DOWN = 3
        BACK = 4
        LEFT = 5
        INVALID = -1

    class TurntablePosition(Enum):
        HOME = 1
        CW = -270
        CCW = 270

    class GrabberPosition(Enum):
        HOME = 1
        GRAB = 170
        FLIP = 40
        REST = 15

    class ScannerPosition(Enum):
        HOME = 1
        SCAN = 20

    def __init__(self):
        self._timer = None
        self._counter = 0
        self._decoder_values = [0, 0, 0]
        self._sensor_values = [0, 0, 0, 0]
        self._nxt = nxt.Remote()
        self._cube_orientation = [0, 1, 2]  # side U, F, R

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
        return self._nxt.connect()

    def disconnect(self):
        return self._nxt.disconnect()

    def start(self):
        self._timer = PeriodicTimer(0.001, self.periodic)

    def stop(self):
        if self._timer is not None:
            self._timer.stop()

        self._nxt.motor_stop(self.MOTOR_TURN)
        self._nxt.motor_stop(self.MOTOR_GRAB)
        self._nxt.motor_stop(self.MOTOR_SCAN)

    def periodic(self):
        self._nxt.poll()
        self._decoder_values[0] = self._nxt.motor_rcv(nxt.PORT_A, 0)
        self._decoder_values[1] = self._nxt.motor_rcv(nxt.PORT_B, 0)
        self._decoder_values[2] = self._nxt.motor_rcv(nxt.PORT_C, 0)
        self._sensor_values[0] = self._nxt.sensor_rcv(nxt.PORT_1, 0)
        self._sensor_values[1] = self._nxt.sensor_rcv(nxt.PORT_2, 0)
        self._sensor_values[2] = self._nxt.sensor_rcv(nxt.PORT_3, 0)
        self._sensor_values[3] = self._nxt.sensor_rcv(nxt.PORT_4, 0)
        self._counter += 1

    def run_to_pos(self, port, speed, position, tolerance):
        cur_pos = self._nxt.motor_rcv(port, 0)
        if cur_pos < position - tolerance:
            self._nxt.motor_cmd(port, speed, position, tolerance)
            while cur_pos < position - tolerance:
                time.sleep(0.001)
                cur_pos = self._nxt.motor_rcv(port, 0)
        elif cur_pos > position + tolerance:
            self._nxt.motor_cmd(port, -speed, position, tolerance)
            while cur_pos > position + tolerance:
                time.sleep(0.001)
                cur_pos = self._nxt.motor_rcv(port, 0)
        # self._nxt.motor_stop(port)

    @staticmethod
    def opposite_face(f):
        if f < 3:
            return f + 3
        return f - 3

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

    def flip_cube(self):
        self.grabber_grab()
        time.sleep(self.REST_TIME)
        self.grabber_flip()
        time.sleep(self.REST_TIME)

    def grab_cube(self):
        self.grabber_grab()
        time.sleep(self.REST_TIME)

    def release_cube(self):
        self.grabber_rest()
        time.sleep(self.REST_TIME)

    def turn_cube(self, num_quarters, with_load=False):
        for i in range(abs(num_quarters)):
            if num_quarters < 0:
                self.turntable_turn_ccw(load=with_load)
            else:
                self.turntable_turn_cw(load=with_load)
            time.sleep(self.REST_TIME)

    def all_home(self):
        self.grabber_home()
        self.scanner_home()
        self.turntable_home()

    def execute_command(self, cmd):
        num_quarter_turns = 1
        if cmd.find("1") != -1:
            num_quarter_turns = 1  # redundant, but okay
        elif cmd.find("2") != -1:
            num_quarter_turns = 2
        if cmd.find("'") != -1:
            num_quarter_turns = -1

        if cmd.find("U") != -1:
            face_to_turn = self.Face.UP.value
        elif cmd.find("F") != -1:
            face_to_turn = self.Face.FRONT.value
        elif cmd.find("R") != -1:
            face_to_turn = self.Face.RIGHT.value
        elif cmd.find("D") != -1:
            face_to_turn = self.Face.DOWN.value
        elif cmd.find("B") != -1:
            face_to_turn = self.Face.BACK.value
        elif cmd.find("L") != -1:
            face_to_turn = self.Face.LEFT.value
        else:
            face_to_turn = self.Face.INVALID.value

        cmd_str = "F{}:N{}:R{}".format(face_to_turn, num_quarter_turns,
                                       self._cube_orientation._str__())

        if face_to_turn == self._cube_orientation[0]:
            # target is top, flip twice
            self.flip_cube()
            self.flip_cube()
            self._cube_orientation[0] = self.opposite_face(self._cube_orientation[0])
            self._cube_orientation[1] = self.opposite_face(self._cube_orientation[1])
        elif face_to_turn == self._cube_orientation[1]:
            # target is front, rotate 180 and flip
            self.release_cube()
            self.turn_cube(2)
            self._cube_orientation[1] = self.opposite_face(self._cube_orientation[1])
            self._cube_orientation[2] = self.opposite_face(self._cube_orientation[2])
            self.flip_cube()
            tmp = self._cube_orientation[0]
            self._cube_orientation[0] = self._cube_orientation[1]
            self._cube_orientation[1] = self.opposite_face(tmp)
        elif face_to_turn == self._cube_orientation[2]:
            # target is right, rotate -90 and flip
            self.release_cube()
            self.turn_cube(1)
            tmp = self._cube_orientation[2]
            self._cube_orientation[2] = self._cube_orientation[1]
            self._cube_orientation[1] = self.opposite_face(tmp)
            self.flip_cube()
            tmp = self._cube_orientation[0]
            self._cube_orientation[0] = self._cube_orientation[1]
            self._cube_orientation[1] = self.opposite_face(tmp)
        elif face_to_turn == self.opposite_face(self._cube_orientation[0]):
            # target is bottom, don't do anything
            pass
        elif face_to_turn == self.opposite_face(self._cube_orientation[1]):
            # target is back, flip
            self.flip_cube()
            tmp = self._cube_orientation[0]
            self._cube_orientation[0] = self._cube_orientation[1]
            self._cube_orientation[1] = self.opposite_face(tmp)
        elif face_to_turn == self.opposite_face(self._cube_orientation[2]):
            # target is left, rotate 90 and flip
            self.release_cube()
            self.turn_cube(1)
            tmp = self._cube_orientation[1]
            self._cube_orientation[1] = self._cube_orientation[2]
            self._cube_orientation[2] = self.opposite_face(tmp)
            self.flip_cube()
            tmp = self._cube_orientation[0]
            self._cube_orientation[0] = self._cube_orientation[1]
            self._cube_orientation[1] = self.opposite_face(tmp)

        self.grab_cube()
        self.turn_cube(num_quarter_turns, True)

        return cmd_str


if __name__ == '__main__':
    sm = SolverMachine()
    sm.connect()


    def handler(signum, frame):
        sm.stop()
        sys.exit()


    signal.signal(signal.SIGINT, handler)

    sm.start()

    sm.all_home()

    time.sleep(1)

    sm.execute_command("U")
    time.sleep(1)
    sm.execute_command("D")
    time.sleep(1)
    sm.execute_command("U'")
    time.sleep(1)
    sm.execute_command("D'")

    time.sleep(10)

    sm.all_home()

    sm.stop()
