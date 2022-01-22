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

from collections import Counter
from collections import deque as RingBuffer

from multiprocessing import Process, Queue
from threading import Thread

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

        self.__received_patterns = 0
        self.__detected_pattern = ''
        self.__current_patterns = RingBuffer(maxlen=10)
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
        self.__console.print_status("RUNNING!")

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
            elif ch == ord('q'):
                break

        self.__console.print_status("STOPPED!")

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
        self.__console.print_values(self.__solver_machine.decoder_values + self.__solver_machine.sensor_values)

        self.__console.update()

    def home(self):
        self.__solver_machine.scanner_home()
        self.__solver_machine.turntable_home()
        self.__solver_machine.grabber_home()

    def flip_cube(self):
        self.__solver_machine.grabber_grab()
        self.__solver_machine.grabber_flip()

    def turn_cube(self, ccw=False):
        if ccw:
            self.__solver_machine.turntable_turn_ccw()
        else:
            self.__solver_machine.turntable_turn_cw()

    def scan_colors(self, side):
        self.__solver_machine.scanner_scan()

        time.sleep(1)

        self.__solver_machine.scanner_home()

    def scan_all_colors(self):
        self.home()
        self.scan_colors('top')
        self.flip_cube()
        self.scan_colors('front')
        self.flip_cube()
        self.scan_colors('bottom')
        self.flip_cube()
        self.turn_cube()
        self.scan_colors('right')
        self.flip_cube()
        self.scan_colors('back')
        self.flip_cube()
        self.scan_colors('left')
        self.home()

    def check_pattern(self, pattern):
        self.__current_patterns.append(pattern)
        self.__received_patterns += 1

        if len(self.__current_patterns) > 0:
            occurrences = Counter(list(self.__current_patterns))
            self.__detected_pattern = occurrences.most_common(1)[0][0]


if __name__ == '__main__':
    cbr = CubeSolver()


    def handler(signum, frame):
        cbr.stop()


    signal.signal(signal.SIGINT, handler)

    cbr.init()
    cbr.start()
    time.sleep(1)
    cbr.exit()
