#!/usr/bin/env python
#
# Copyright (c) 2021 Timon Reich
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# This Python application will use my NXT remote control library with its Python binding `nxt_remote_py`
# to control the Lego model gathered from the MindCuber page (see http://mindcuber.com/).

import sys
import logging
import time

from nxt_remote_libs import periodic_timer as timer

import nxt_remote_py as nxt


def hello(name):
    print("Hello %s!" % name)


logging.basicConfig(level=logging.INFO)

logging.info('Connecting to NXT...')
rc = nxt.Remote()

if not rc.connect():
    logging.error('Could not connect!')
    sys.exit(1)

rt = timer.PeriodicTimer(1, hello, "World")

try:
    time.sleep(5)
finally:
    rt.stop()

if not rc.disconnect():
    logging.error('Could not disconnect!')
    sys.exit(1)

logging.info('Done.')
sys.exit(0)
