#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import cv2
import socket
import pickle
import struct

import camera_source as camera


class VideoSender(object):

    def __init__(self, host, port):
        self._host = host
        self._port = port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._file = self._socket.makefile('wb')

    def connect(self):
        self._socket.connect((self._host, self._port))

    def send(self, img):
        if self._file is None:
            return None

        data = pickle.dumps(img)
        header = len(data).to_bytes(4,'big')

        self._file.write(header)
        self._file.write(data)
        self._file.flush()

    def close(self):
        self._socket.close()


if __name__ == '__main__':

    host = 'localhost'
    port = 1234
    ss = VideoSender(host, port)
    ss.connect()

    address = '/dev/video0'
    cs = camera.CameraSource(address, 1280, 720)
    cs.open()

    while True:
        img = cs.read()
        if img is not None:
            ss.send(img)
            # cv2.imshow('original', img)
        if cs.exited():
            ss.close()
            break
