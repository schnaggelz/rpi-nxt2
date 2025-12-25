import rclpy
import sensor_msgs

from sensor_msgs.msg import Image, CompressedImage
from rclpy.node import Node
from rclpy.parameter import Parameter
from cv_bridge import CvBridge

from  picamera2 import Picamera2, Preview
import cv2
import numpy as np

import queue
import threading
import time


class CamNode(Node):

    def __init__(self):
        super().__init__('cam_node')

        # Create publisher for raw images
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)

        # Bridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Initialize Picamera2
        self.picam2 = Picamera2()

        # Configure camera (adjust resolution & format as needed)
        camera_config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam2.configure(camera_config)
        self.picam2.start()

        # Publish at ~30 FPS
        self.timer = self.create_timer(1.0 / 30.0, self.capture_and_publish)

        self.get_logger().info("RPi Camera Node started, publishing to /camera/image_raw")

    def capture_and_publish(self):
        try:
            # Capture frame as numpy array
            frame = self.picam2.capture_array()

            # Convert to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')

            # Publish
            self.publisher_.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Camera capture failed: {e}")

    def destroy_node(self):
        self.picam2.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
