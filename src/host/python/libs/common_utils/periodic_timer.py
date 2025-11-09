#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

from threading import Timer

class PeriodicTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self._interval = interval
        self._function = function
        self._args = args
        self._kwargs = kwargs
        self._is_running = False
        self.start()

    def _run(self):
        self._is_running = False
        self.start()
        self._function(*self._args, **self._kwargs)

    def start(self):
        if not self._is_running:
            self._timer = Timer(self._interval, self.__run)
            self._timer.start()
            self._is_running = True

    def stop(self):
        self._timer.cancel()
        self._is_running = False
