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

import nxt_remote_py as nxt

logging.basicConfig(level=logging.INFO)

logging.info('Connecting to NXT...')
r = nxt.Remote()

if not r.connect():
    logging.error('Could not connect!')
    sys.exit(1)

logging.warning('Playing a bit with the NXT...')

if not r.motor_fwd(nxt.PORT_A, 50):
    logging.error('Could not control motor A!')
    sys.exit(1)

time.sleep(1)
if not r.motor_rev(nxt.PORT_A, 50):
    logging.error('Could not control motor A!')
    sys.exit(1)

time.sleep(1)
if not r.motor_stop(nxt.PORT_A):
    logging.error('Could not control motor A!')
    sys.exit(1)

time.sleep(1)
if not r.motor_fwd(nxt.PORT_B, 50):
    logging.error('Could not control motor B!')
    sys.exit(1)

time.sleep(1)
if not r.motor_rev(nxt.PORT_B, 50):
    logging.error('Could not control motor B!')
    sys.exit(1)

time.sleep(1)
if not r.motor_stop(nxt.PORT_B):
    logging.error('Could not control motor B!')
    sys.exit(1)

time.sleep(1)

logging.info('Disconnecting from NXT...')
if not r.disconnect():
    logging.error('Could not disconnect!')
    sys.exit(1)

logging.info('Done.')
sys.exit(0)
