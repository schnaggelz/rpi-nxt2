import numpy as np
import cv2

from flask import Flask, Response

class VisionServer:
    def __init__(self, vision_system, host='0.0.0.0', port=5000):
        self._app = Flask(__name__)
        self._vision = vision_system
        self._host = host
        self._port = port

        # Register routes
        self._app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self._app.add_url_rule('/', 'index', self.index)

    def index(self):
        return "<h1>NXT Robot Live Feed</h1><img src='/video_feed' width='640'>"

    def video_feed(self):
        return Response(self._generate_frames(), 
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    def _generate_frames(self):
        while True:
            if self._vision is None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + 
                       self._create_placeholder_frame() + b'\r\n')
                continue
            frame = self._vision.get_frame()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def _create_placeholder_frame(self):
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, 'No Video Feed', (50, 240), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        _, jpeg = cv2.imencode('.jpg', placeholder)
        return jpeg.tobytes()

    def run(self):
        self._app.run(host=self._host, port=self._port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    server = VisionServer(None)
    server.run()
