#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import zmq
import base64
import numpy as np
import cv2

class VideoReceiver(object):

    def __init__(self, port):
        context = zmq.Context()
        self._port = port
        self._socket = context.socket(zmq.SUB)

    def connect(self):
        self._socket.bind("tcp://*:{}".format(self._port))
        self._socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

    def receive(self):
        buffer = self._socket.recv_string()
        data = base64.b64decode(buffer)

        npimg = np.fromstring(data, dtype=np.uint8)
        img = cv2.imdecode(npimg, 1)

        return img


if __name__ == '__main__':

    receiver = VideoReceiver(1234)
    receiver.connect()

    while True:
        img = receiver.receive()
        cv2.imshow('received', img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
