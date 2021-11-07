#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import nxt_remote_py


class CubeSolver(object):
    nxt = nxt_remote_py.Remote()

    def __init__(self, debug=False):
        self.debug = debug

    def setup(self):
        if not nxt.connect():
            return False #TODO



