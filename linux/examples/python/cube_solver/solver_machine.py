#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import time
import nxt_remote_py as nxt

from enum import Enum

from common_utils.periodic_timer import PeriodicTimer


class SolverMachine:
    VERSION = 1

    # Motor speed constants
    MOTOR_GRAB_SPEED = 50
    MOTOR_TURN_SPEED = 100
    MOTOR_SCAN_SPEED = 50

    MOTOR_TURN = nxt.PORT_A
    MOTOR_SCAN = nxt.PORT_B
    MOTOR_GRAB = nxt.PORT_C

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
        self.__timer = None
        self.__counter = 0
        self.__decoder_values = [0, 0, 0]
        self.__sensor_values = [0, 0, 0, 0]
        self.__nxt = nxt.Remote()
        self.__cube_orientation = [0, 1, 2]  # side U, F, R

    @property
    def decoder_values(self):
        return self.__decoder_values

    @property
    def sensor_values(self):
        return self.__sensor_values

    @property
    def counter(self):
        return self.__counter

    def connect(self):
        return self.__nxt.connect()

    def disconnect(self):
        return self.__nxt.disconnect()

    def start(self):
        self.__timer = PeriodicTimer(0.001, self.periodic)

    def stop(self):
        if self.__timer is not None:
            self.__timer.stop()

        self.__nxt.motor_stop(self.MOTOR_TURN)
        self.__nxt.motor_stop(self.MOTOR_GRAB)
        self.__nxt.motor_stop(self.MOTOR_SCAN)

    def periodic(self):
        self.__nxt.poll()
        self.__decoder_values[0] = self.__nxt.motor_rcv(nxt.PORT_A, 0)
        self.__decoder_values[1] = self.__nxt.motor_rcv(nxt.PORT_B, 0)
        self.__decoder_values[2] = self.__nxt.motor_rcv(nxt.PORT_C, 0)
        self.__sensor_values[0] = self.__nxt.sensor_rcv(nxt.PORT_1, 0)
        self.__sensor_values[1] = self.__nxt.sensor_rcv(nxt.PORT_2, 0)
        self.__sensor_values[2] = self.__nxt.sensor_rcv(nxt.PORT_3, 0)
        self.__sensor_values[3] = self.__nxt.sensor_rcv(nxt.PORT_4, 0)
        self.__counter += 1

    def run_to_pos(self, port, speed, position, tolerance):
        cur_pos = self.__nxt.motor_rcv(port, 0)
        if cur_pos < position - tolerance:
            self.__nxt.motor_cmd(port, speed, position, tolerance)
            while cur_pos < position - tolerance:
                time.sleep(0.001)
                cur_pos = self.__nxt.motor_rcv(port, 0)
        elif cur_pos > position + tolerance:
            self.__nxt.motor_cmd(port, -speed, position, tolerance)
            while cur_pos > position + tolerance:
                time.sleep(0.001)
                cur_pos = self.__nxt.motor_rcv(port, 0)
        # self.__nxt.motor_stop(port)

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
        self.grabber_flip()

    def release_cube(self):
        self.grabber_rest()

    def turn_cube(self, num_quarters, with_load=False):
        for i in range(abs(num_quarters)):
            if num_quarters < 0:
                self.turntable_turn_ccw(load=with_load)
            else:
                self.turntable_turn_cw(load=with_load)

    def execute_command(self, cmd):
        face_to_turn = None
        num_quarter_turns = 1
        if cmd.find("2") != -1:
            num_quarter_turns = 2
        if cmd.find("'") != -1:
            num_quarter_turns = -1
        
        if cmd.find("U") != -1:
            face_to_turn = 0
        elif cmd.find("F") != -1:
            face_to_turn = 1
        elif cmd.find("R") != -1:
            face_to_turn = 2
        elif cmd.find("D") != -1:
            face_to_turn = 3
        elif cmd.find("B") != -1:
            face_to_turn = 4
        elif cmd.find("L") != -1:
            face_to_turn = 5
        
        if face_to_turn == self.__cube_orientation[0]:
            # target is top, flip twice
            self.flip_cube()
            self.flip_cube()
            self.__cube_orientation[0] = self.opposite_face(self.__cube_orientation[0])
            self.__cube_orientation[1] = self.opposite_face(self.__cube_orientation[1])
        elif face_to_turn == self.__cube_orientation[1]:
            # target is front, rotate 180 and flip
            self.release_cube()
            self.turn_cube(2)
            self.__cube_orientation[1] = self.opposite_face(self.__cube_orientation[1])
            self.__cube_orientation[2] = self.opposite_face(self.__cube_orientation[2])
            self.flip_cube()
            tmp = self.__cube_orientation[0]
            self.__cube_orientation[0] = self.__cube_orientation[1]
            self.__cube_orientation[1] = self.opposite_face(tmp)
        elif face_to_turn == self.__cube_orientation[2]:
            # target is right, rotate -90 and flip
            self.release_cube()
            self.turn_cube(1)
            tmp = self.__cube_orientation[2]
            self.__cube_orientation[2] = self.__cube_orientation[1]
            self.__cube_orientation[1] = self.opposite_face(tmp)
            self.flip_cube()
            tmp = self.__cube_orientation[0]
            self.__cube_orientation[0] = self.__cube_orientation[1]
            self.__cube_orientation[1] = self.opposite_face(tmp)
        elif face_to_turn == self.opposite_face(self.__cube_orientation[0]):
            # target is bottom, don't do anything
            pass
        elif face_to_turn == self.opposite_face(self.__cube_orientation[1]):
            # target is back, flip
            self.flip_cube()
            tmp = self.__cube_orientation[0]
            self.__cube_orientation[0] = self.__cube_orientation[1]
            self.__cube_orientation[1] = self.opposite_face(tmp)
        elif face_to_turn == self.opposite_face(self.__cube_orientation[2]):
            # target is left, rotate 90 and flip
            self.release_cube()
            self.turn_cube(1)
            tmp = self.__cube_orientation[1]
            self.__cube_orientation[1] = self.__cube_orientation[2]
            self.__cube_orientation[2] = self.opposite_face(tmp)
            self.flip_cube()
            tmp = self.__cube_orientation[0]
            self.__cube_orientation[0] = self.__cube_orientation[1]
            self.__cube_orientation[1] = self.opposite_face(tmp)

        self.grab()
        self.turn_cube(num_quarter_turns, True)


if __name__ == '__main__':
    sm = SolverMachine()
