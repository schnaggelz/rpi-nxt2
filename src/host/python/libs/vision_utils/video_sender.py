#!/usr/bin/env python
#
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
#
# Utility code
#

import base64
import cv2
import zmq


class VideoSender(object):

    def __init__(self, host, port):
        context = zmq.Context()
        self._host = host
        self._port = port
        self._socket = context.socket(zmq.PUB)

    def connect(self):
        self._socket.connect("tcp://{}:{}".format(self._host, self._port))

    def send(self, img):
        rgb_img = cv2.cvtColor(img, cv2.COLOR_YUV420p2RGB)
        encoded, data = cv2.imencode('.jpg', rgb_img)
        buffer = base64.b64encode(data)
        try:
            self._socket.send(buffer, zmq.NOBLOCK)
        except zmq.ZMQError:
            pass


if __name__ == '__main__':
    import pi_camera

    sender = VideoSender('192.168.242.163', 4243)
    sender.connect()

    def receive(img):
        sender.send(img)

    camera = pi_camera.Camera(lores_size=(320, 240),
                              lores_callback=receive)
    camera.open()
    camera.run()
