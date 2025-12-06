import rclpy

from rclpy.node import Node
from nxt_msgs.msg import MotorCommand
from nxt_msgs.msg import Ports

class MotorCommandTestNode(Node):
    def __init__(self):
        super().__init__('motor_command_test')
        
        self.pub = self.create_publisher(
            MotorCommand,
            'nxt/motor_cmd',
            10
        )
        
        self.timer = self.create_timer(5.0, self.publish_commands)
        self.cycle = 0
        
        self.get_logger().info('Motor command test node started')


    def publish_commands(self):

        msg = MotorCommand()
        step = self.cycle % 4

        if step == 0:
            # Motor A: speed 80, count 200 ticks, tolerance 5
            msg.port = Ports.PORT_A
            msg.speed = 80
            msg.count = 200
            msg.tolerance = 5
            self.get_logger().info('Sending: PORT A, speed=80, count=200, tol=5')
            self.pub.publish(msg)
        elif step == 1:
            # Motor B reverse: speed -60, continuous (count=0)
            msg.port = Ports.PORT_B
            msg.speed = -60
            msg.count = 0
            msg.tolerance = 0
            self.get_logger().info('Sending: PORT B, speed=-60 (continuous)')
            self.pub.publish(msg)
        elif step == 2:
            # Motor C: speed 50, count 1000, tolerance 10
            msg.port = Ports.PORT_C
            msg.speed = 50
            msg.count = 1000
            msg.tolerance = 10
            self.get_logger().info('Sending: PORT C, speed=50, count=1000, tol=10')
            self.pub.publish(msg)
        else:
            self.get_logger().info('Stopping all motors')
            for i in range(Ports.NUM_MOTOR_PORTS):
                msg.port = i
                msg.speed = 0
                msg.count = 0
                msg.tolerance = 0
                self.pub.publish(msg)

        self.cycle += 1


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()