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
import queue
import numpy as np
import kociemba

from collections import deque as RingBuffer

from multiprocessing import Process, Queue
from threading import Thread
from threading import Event

from common_utils.periodic_timer import PeriodicTimer
from vision_utils.video_sender import VideoSender

from solver_machine import SolverMachine
from color_detector import ColorDetector
from solver_console import SolverConsole


class CubeSolver:
    VERSION = 1

    class DetectorReceiver(Thread):

        def __init__(self, queue, callback):
            super().__init__()
            self.__queue = queue
            self.__callback = callback
            self.__stop = False

        def run(self):
            while not self.__stop:
                try:
                    value = self.__queue.get(timeout=1)
                    self.__callback(value)
                except queue.Empty:
                    pass

        def stop(self):
            self.__stop = True

    @staticmethod
    def run_detector(output):
        video_sink = VideoSender('192.168.242.137', 4243)  # configurable?
        video_sink.connect()

        detector = ColorDetector(output=output)
        detector.video_sink = video_sink
        detector.start()

    def __init__(self):
        self.__console = SolverConsole()
        self.__solver_machine = SolverMachine()

        self.__detected_event = Event()
        self.__detected_patterns = dict.fromkeys(['U', 'L', 'F', 'R', 'B', 'D'])
        self.__current_patterns = RingBuffer(maxlen=10)
        self.__current_side = None
        self.__patterns = Queue(maxsize=10)

        self.__detector_process = Process(target=CubeSolver.run_detector,
                                          args=(self.__patterns,))

        self.__detector_receiver = self.DetectorReceiver(
            queue=self.__patterns,
            callback=self.check_pattern)

        self.__timer = PeriodicTimer(0.1, self.display)

    def init(self):
        if self.__solver_machine.connect():
            self.__console.print_status("CONNECTED")
        else:
            self.__console.print_status("NOT CONNECTED!")

    def exit(self):
        self.__solver_machine.disconnect()

    def start(self):
        self.__detector_process.start()
        self.__detector_receiver.start()
        self.__solver_machine.start()

        while True:
            time.sleep(0.5)
            ch = self.__console.get_char()
            if ch == ord('h'):
                self.__solver_machine.turntable_home()
            if ch == ord('e'):
                self.__solver_machine.turntable_turn_ccw()
            if ch == ord('w'):
                self.__solver_machine.turntable_turn_cw()
            if ch == ord('z'):
                self.__solver_machine.turntable_home(True)
            if ch == ord('x'):
                self.__solver_machine.turntable_turn_ccw(True)
            if ch == ord('y'):
                self.__solver_machine.turntable_turn_cw(True)
            if ch == ord('o'):
                self.__solver_machine.grabber_home()
            if ch == ord('r'):
                self.__solver_machine.grabber_rest()
            if ch == ord('g'):
                self.__solver_machine.grabber_grab()
            if ch == ord('f'):
                self.__solver_machine.grabber_flip()
            if ch == ord('c'):
                self.__solver_machine.scanner_scan()
            if ch == ord('m'):
                self.__solver_machine.scanner_home()
            if ch == ord('s'):
                self.scan_all_colors()
            if ch == ord('r'):
                self.solve_cube()
            elif ch == ord('q'):
                break

        self.stop()

    def stop(self):
        self.__detector_receiver.stop()
        self.__detector_receiver.join(2)

        self.__detector_process.terminate()
        self.__detector_process.join(2)

        self.__solver_machine.stop()
        self.__timer.stop()

    def display(self):
        self.__console.print_version("CUBER", self.VERSION)

        self.__console.print_counter(self.__solver_machine.counter)
        self.__console.print_values(self.__solver_machine.decoder_values +
                                    self.__solver_machine.sensor_values)

        self.__console.update()

    def scan_colors(self, side):
        scan_success = False

        self.__current_side = side
        self.__detected_patterns[side] = None
        self.__current_patterns.clear()
        self.__detected_event.clear()

        self.__solver_machine.scanner_scan()

        counter = 0
        while counter < 50:  # max 5 seconds
            self.__detected_event.wait(0.1)
            counter += 1

        if self.__detected_event.is_set():
            scan_success = True
            self.__console.print_cube_notation(side, self.__detected_patterns[side])

        self.__solver_machine.scanner_home()

        return scan_success

    def scan_all_colors(self):
        scan_success = True

        self.home()
        if not self.scan_colors('U'):  # red (top)
            scan_success = False

        self.__solver_machine.flip_cube()
        if not self.scan_colors('F'):  # green (front)
            scan_success = False

        self.__solver_machine.flip_cube()
        if not self.scan_colors('D'):  # orange (bottom)
            scan_success = False

        self.__solver_machine.turn_cube(-1)
        self.__solver_machine.flip_cube()
        self.__solver_machine.turn_cube(1)
        if not self.scan_colors('L'):  # white (left)
            scan_success = False

        self.__solver_machine.flip_cube()
        if not self.scan_colors('B'):  # blue (back)
            scan_success = False

        self.__solver_machine.flip_cube()
        if not self.scan_colors('R'):  # yellow (right)
            scan_success = False

        self.__solver_machine.flip_cube()
        self.__solver_machine.all_home()

        if scan_success:
            self.__console.print_status("SCAN SUCCESS")
        else:
            self.__console.print_status("SCAN FAILURE!")

        return scan_success

    def check_pattern(self, pattern):
        self.__current_patterns.append(pattern)

        max_detect = 0
        if len(self.__current_patterns) == self.__current_patterns.maxlen:
            unique_patterns, counts = np.unique(
                self.__current_patterns, axis=0, return_counts=True)
            max_indexes = np.where(counts == np.amax(counts))[0]

            if len(max_indexes) == 1:
                max_index = max_indexes[0]

                if counts[max_index] > 8:
                    max_detect = counts[max_index]
                    self.__console.print_cube_notation('?', unique_patterns[max_index])
                    self.__detected_patterns[self.__current_side] = unique_patterns[max_index]
                    self.__detected_event.set()

        self.__console.print_max_detect(max_detect)

    def solve_cube(self):
        patterns = ''
        patterns += ''.join(self.__detected_patterns['U'])
        patterns += ''.join(self.__detected_patterns['R'])
        patterns += ''.join(self.__detected_patterns['F'])
        patterns += ''.join(self.__detected_patterns['D'])
        patterns += ''.join(self.__detected_patterns['L'])
        patterns += ''.join(self.__detected_patterns['B'])

        cmds = kociemba.solve(patterns)
        for cmd in cmds.split():
            self.__console.print_command(cmd)
            self.__solver_machine.execute_command(cmd)


if __name__ == '__main__':
    cbr = CubeSolver()


    def handler(signum, frame):
        cbr.stop()


    signal.signal(signal.SIGINT, handler)

    cbr.init()
    cbr.start()
    time.sleep(1)
    cbr.exit()
