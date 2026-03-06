import numpy as np
import cv2

from picamera2 import Picamera2
from picamera2.devices import Hailo

class RobotVision:
    def __init__(self, model_path):
        self._camera = Picamera2()
        self._hailo = Hailo(model_path)
        model_h, model_w, _ = self._hailo.get_input_shape()
        config = self._camera.create_preview_configuration(
            main={"size": (model_h, model_w), "format": "RGB888"})
        self._camera.configure(config)

    def get_frame(self):
        raw_frame = self._camera.capture_array()
        inference_results = self._hailo.run(raw_frame)
        ret, compressed_frame = cv2.imencode('.jpg', raw_frame)
        return compressed_frame.tobytes()

    def start(self):
        self._camera.start()

if __name__ == "__main__":
    vision = RobotVision(model_path="/usr/share/hailo-models/yolov5n_seg_h8.hef")
    vision.start()
