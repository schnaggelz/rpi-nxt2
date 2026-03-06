from robot_vision import RobotVision
from video_server import VisionServer

if __name__ == "__main__":
    model_path = "/usr/share/hailo-models/yolov5n_seg_h8.hef"
    vision = RobotVision(model_path=model_path)
    vision.start()
    server = VisionServer(vision)
    server.run()
