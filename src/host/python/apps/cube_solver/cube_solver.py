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
            super()._init__()
            self._queue = queue
            self._callback = callback
            self._stop = False

        def run(self):
            while not self._stop:
                try:
                    value = self._queue.get(timeout=1)
                    self._callback(value)
                except queue.Empty:
                    pass

        def stop(self):
            self._stop = True

    @staticmethod
    def run_detector(output):
        video_sink = VideoSender('192.168.242.137', 4243)  # configurable?
        video_sink.connect()

        detector = ColorDetector(output=output)
        detector.video_sink = video_sink
        detector.start()

    def __init__(self):
        self._console = SolverConsole()
        self._solver_machine = SolverMachine()

        self._detected_event = Event()
        self._detected_patterns = dict.fromkeys(['U', 'L', 'F', 'R', 'B', 'D'])
        self._current_patterns = RingBuffer(maxlen=10)
        self._current_side = None
        self._patterns = Queue(maxsize=10)

        self._detector_process = Process(target=CubeSolver.run_detector,
                                          args=(self._patterns,))

        self._detector_receiver = self.DetectorReceiver(
            queue=self._patterns,
            callback=self.check_pattern)

        self._timer = PeriodicTimer(0.1, self.display)

    def init(self):
        self._console.setup()
        if self._solver_machine.connect():
            self._console.print_status("CONNECTED")
        else:
            self._console.print_status("NOT CONNECTED!")

    def exit(self):
        self._solver_machine.disconnect()

    def start(self):
        self._detector_process.start()
        self._detector_receiver.start()
        self._solver_machine.start()

        while True:
            time.sleep(0.5)
            ch = self._console.get_char()
            if ch == ord('h'):
                self._solver_machine.turntable_home()
            elif ch == ord('e'):
                self._solver_machine.turntable_turn_ccw()
            elif ch == ord('w'):
                self._solver_machine.turntable_turn_cw()
            elif ch == ord('z'):
                self._solver_machine.turntable_home(True)
            elif ch == ord('x'):
                self._solver_machine.turntable_turn_ccw(True)
            elif ch == ord('y'):
                self._solver_machine.turntable_turn_cw(True)
            elif ch == ord('o'):
                self._solver_machine.grabber_home()
            elif ch == ord('r'):
                self._solver_machine.grabber_rest()
            elif ch == ord('g'):
                self._solver_machine.grabber_grab()
            elif ch == ord('f'):
                self._solver_machine.grabber_flip()
            elif ch == ord('p'):
                self._solver_machine.scanner_scan()
            elif ch == ord('m'):
                self._solver_machine.scanner_home()
            elif ch == ord('s'):
                self.scan_all_colors()
            elif ch == ord('r'):
                self.solve_cube()
            elif ch == ord('c'):
                self.command_mode()
            elif ch == ord('q'):
                break

        self.stop()

    def stop(self):
        self._detector_receiver.stop()
        self._detector_receiver.join(2)

        self._detector_process.terminate()
        self._detector_process.join(2)

        self._solver_machine.stop()
        self._timer.stop()

    def display(self):
        self._console.print_version("CUBER", self.VERSION)

        self._console.print_counter(self._solver_machine.counter)
        self._console.print_values(self._solver_machine.decoder_values +
                                    self._solver_machine.sensor_values)

        self._console.update()

    def scan_colors(self, side):
        scan_success = False

        self._current_side = side
        self._detected_patterns[side] = None
        self._current_patterns.clear()
        self._detected_event.clear()

        self._solver_machine.scanner_scan()

        counter = 0
        while counter < 50:  # max 5 seconds
            self._detected_event.wait(0.1)
            counter += 1

        if self._detected_event.is_set():
            scan_success = True
            self._console.print_cube_notation(side, self._detected_patterns[side])

        self._solver_machine.scanner_home()

        return scan_success

    def scan_all_colors(self):
        scan_success = True

        self.home()
        if not self.scan_colors('U'):  # red (top)
            scan_success = False

        self._solver_machine.flip_cube()
        if not self.scan_colors('F'):  # green (front)
            scan_success = False

        self._solver_machine.flip_cube()
        if not self.scan_colors('D'):  # orange (bottom)
            scan_success = False

        self._solver_machine.turn_cube(-1)
        self._solver_machine.flip_cube()
        self._solver_machine.turn_cube(1)
        if not self.scan_colors('L'):  # white (left)
            scan_success = False

        self._solver_machine.flip_cube()
        if not self.scan_colors('B'):  # blue (back)
            scan_success = False

        self._solver_machine.flip_cube()
        if not self.scan_colors('R'):  # yellow (right)
            scan_success = False

        self._solver_machine.flip_cube()
        self._solver_machine.all_home()

        if scan_success:
            self._console.print_status("SCAN SUCCESS")
        else:
            self._console.print_status("SCAN FAILURE!")

        return scan_success

    def check_pattern(self, pattern):
        self._current_patterns.append(pattern)

        max_detect = 0
        if len(self._current_patterns) == self._current_patterns.maxlen:
            unique_patterns, counts = np.unique(
                self._current_patterns, axis=0, return_counts=True)
            max_indexes = np.where(counts == np.amax(counts))[0]

            if len(max_indexes) == 1:
                max_index = max_indexes[0]

                if counts[max_index] > 8:
                    max_detect = counts[max_index]
                    self._console.print_cube_notation('?', unique_patterns[max_index])
                    self._detected_patterns[self._current_side] = unique_patterns[max_index]
                    self._detected_event.set()

        self._console.print_max_detect(max_detect)

    def solve_cube(self):
        patterns = ''
        patterns += ''.join(self._detected_patterns['U'])
        patterns += ''.join(self._detected_patterns['R'])
        patterns += ''.join(self._detected_patterns['F'])
        patterns += ''.join(self._detected_patterns['D'])
        patterns += ''.join(self._detected_patterns['L'])
        patterns += ''.join(self._detected_patterns['B'])

        cmds = kociemba.solve(patterns)
        for cmd in cmds.split():
            self._console.print_command(cmd)
            self._solver_machine.execute_command(cmd)

    def command_mode(self):
        cmd = self._console.get_command()
        self._console.print_command(cmd)
        res = self._solver_machine.execute_command(cmd)
        self._console.print_result(res)


if __name__ == '__main__':
    cbr = CubeSolver()


    def handler(signum, frame):
        cbr.stop()


    signal.signal(signal.SIGINT, handler)

    cbr.init()
    cbr.start()
    time.sleep(1)
    cbr.exit()
