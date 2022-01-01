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


class VideoReceiver(object):
    PCK_RCV_SIZE = 4096
    PCK_HDR_SIZE = struct.calcsize('Q')

    def __init__(self, host, port):
        self._host = host
        self._port = port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.bind((self._host, self._port))
        self._socket.listen(10)
        self._file = None

    def connect(self):
        connection, _ = self._socket.accept()
        if connection:
            self._file = connection.makefile('rb')

    def receive(self):
        if self._file is None:
            return None

        header = self._file.read(4)
        if not header:
            return None

        data_size = int.from_bytes(header, 'big')
        data = self._file.read(data_size)

        return pickle.loads(data)

    def close(self):
        self._socket.close()

if __name__ == '__main__':

    host = 'localhost'
    port = 1234
    sr = VideoReceiver(host, port)
    sr.connect()

    while True:
        img = sr.receive()
        cv2.imshow('received', img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break