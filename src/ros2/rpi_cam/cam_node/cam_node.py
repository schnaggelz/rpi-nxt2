import picamera2

import rclpy
import sensor_msgs

from sensor_msgs.msg import Image, CompressedImage
from rclpy.node import Node
from rclpy.parameter import Parameter

class CamNode(Node):

    def __init__(self):
        super().__init__('cam_node', namespace='rpi_cam')

        self.camera = picamera2.Picamera2()
        self.create_timer(0.2, self.timer_callback)
        self.create_publishers()

    def timer_callback(self):
        self.get_logger().info("Hello ROS2")

    def destroy_node(self):
        if hasattr(self, 'camera') and self.camera != None:
            self.camera.close()
        super().destroy_node()

    def create_publishers(self):
        self.publisher = self.create_publisher(Image, 'picam_uncompressed')
        self.frame_num = 0

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

