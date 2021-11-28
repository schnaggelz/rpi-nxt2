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
import signal

from nxt_remote_libs import periodic_timer as timer
from nxt_remote_libs import text_printer as printer

import nxt_remote_py as nxt

stop = False


def handler(signum, frame):
    global stop
    stop = True


signal.signal(signal.SIGINT, handler)

p = printer.TextPrinter(sys.stdout)


def poll(r):
    r.poll()
    p.print_at(10, 2, "M_A: {}".format(str(rc.motor_rcv(nxt.PORT_A, 0))))
    p.print_at(11, 2, "M_B: {}".format(str(rc.motor_rcv(nxt.PORT_B, 0))))
    p.print_at(12, 2, "M_C: {}".format(str(rc.motor_rcv(nxt.PORT_C, 0))))


logging.basicConfig(level=logging.INFO)

logging.info('Connecting to NXT...')
rc = nxt.Remote()

if not rc.connect():
    logging.error('Could not connect!')
    sys.exit(1)

rt = timer.PeriodicTimer(1, poll, rc)

if not rc.motor_fwd(nxt.PORT_A, 20):
    logging.error('Could not control motor A!')

try:
    while not stop:
        time.sleep(0.1)
finally:
    rt.stop()


if not rc.motor_stop(nxt.PORT_A):
    logging.error('Could not control motor A!')

if not rc.disconnect():
    logging.error('Could not disconnect!')

logging.info('Done.')
sys.exit(0)
