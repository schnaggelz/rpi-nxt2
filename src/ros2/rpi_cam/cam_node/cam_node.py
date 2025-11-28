import rclpy
import sensor_msgs

from sensor_msgs.msg import Image, CompressedImage
from rclpy.node import Node
from rclpy.parameter import Parameter

import picamera2

import queue
import threading
import time


class CamNode(Node):

    def __init__(self):
        super().__init__('cam_node', namespace='rpi_cam')

        self.camera = picamera2.Picamera2()
        self.create_publishers()
        self.start_camera()
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        self.publish_frame()

    def destroy_node(self):
        if hasattr(self, 'camera') and self.camera != None:
            self.camera.close()
        super().destroy_node()

    def create_publishers(self):
        self.publisher = self.create_publisher(Image, 'picam_uncompressed', 2)
        self.frame_num = 0

    def start_camera(self):
        self.camera.configure(self.camera.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}))
        self.camera.start()

    def publish_frame(self):
        frame = self.camera.capture_array()
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'picam_frame'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'rgba8'
        msg.is_bigendian = 0
        msg.step = frame.strides[0]
        msg.data = frame.tobytes()
        
        self.publisher.publish(msg)
        self.frame_num += 1

def main(args=None):

    rclpy.init(args=args)
    node = CamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # node.get_logger().info('Keyboard interrupt')

    rclpy.shutdown()

if __name__ == '__main__':
    main()

