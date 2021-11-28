#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

class TextPrinter(object):
    def __init__(self, stream):
        self._stream = stream

    def print_at(self, x, y, text):
        self._stream.write("\x1b7\x1b[%d;%df%s\x1b8" % (x, y, text))
        self._stream.flush()
