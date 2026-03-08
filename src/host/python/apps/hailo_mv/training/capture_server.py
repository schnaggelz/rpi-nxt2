import argparse
import os

import numpy as np
import cv2

from picamera2 import Picamera2
from flask import Flask, Response

class CaptureServer:
    def __init__(self, output_path='.', host='0.0.0.0', port=5000):
        self._app = Flask(__name__)
        self._camera = Picamera2()
        self._host = host
        self._port = port

        config = self._camera.create_preview_configuration(
        main={"size": (480, 640), "format": "RGB888"})
        self._camera.configure(config)

        # Register routes
        self._app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self._app.add_url_rule('/', 'index', self.index)

        self._output_path = output_path
        self._counter = 0

    def index(self):
        return "<h1>NXT Robot Capture Feed</h1><img src='/video_feed' width='640'>"

    def video_feed(self):
        return Response(self._generate_frames(), 
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    def _generate_frames(self):
        while True:
            if self._camera is None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + 
                       self._create_placeholder_frame() + b'\r\n')
                continue
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + self._request_capture_frame() + b'\r\n')

    def _create_placeholder_frame(self):
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, 'No Video Feed', (50, 240), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        _, jpeg = cv2.imencode('.jpg', placeholder)
        return jpeg.tobytes()
    
    def _request_capture_frame(self):
        if self._camera is None:
            return None
        
        cv2.waitKey(1) & 0xFF

        path = os.path.join(self._output_path, f"img_{self._counter:05d}.jpg")
        raw_frame = self._camera.capture_array()
        
        cv2.imwrite(path, raw_frame)
        print(f"Saved: {path}")
        self._counter += 1

        _, compressed_frame = cv2.imencode('.jpg', raw_frame)
        return compressed_frame.tobytes()

    def run(self):
        self._camera.start()
        self._app.run(host=self._host, port=self._port, threaded=True, use_reloader=False)

    def exit(self):
        if self._camera is not None:
            self._camera.stop()
            self._camera = None

def main():
    parser = argparse.ArgumentParser(description="Capture training images from Pi camera")
    parser.add_argument("--output", type=str, default="dataset/images", help="Output directory")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=640)
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    print(f"Saving to: {args.output}")
    print("SPACE = capture")

    server = CaptureServer(output_path=args.output)
    try:
        server.run()
    finally:
        server.exit()


if __name__ == "__main__":
    main()

