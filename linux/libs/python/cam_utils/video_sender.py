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
        encoded, data = cv2.imencode('.jpg', img)
        buffer = base64.b64encode(data)
        try:
            self._socket.send(buffer, zmq.NOBLOCK)
        except zmq.ZMQError:
            pass

if __name__ == '__main__':

    from cam_utils.pi_camera import Camera

    sender = VideoSender('treich-dt-1', 1234)
    sender.connect()

    def receive(img):
        sender.send(img)

    camera = Camera(image_width=640,
                    image_height=480,
                    framerate=10,
                    read_callback=receive)
    camera.open()
    camera.run()
