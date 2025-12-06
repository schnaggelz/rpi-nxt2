import rclpy
from rclpy.node import Node
from nxt_msgs.msg import SimpleMotorCommand


class MotorCommandTestNode(Node):
    def __init__(self):
        super().__init__('motor_command_test')
        
        self.pub = self.create_publisher(
            SimpleMotorCommand,
            'nxt/simple_motor_cmd',
            10
        )
        
        self.timer = self.create_timer(2.0, self.publish_commands)
        self.cycle = 0
        
        self.get_logger().info('Motor command test node started')

    def publish_commands(self):
        msg = SimpleMotorCommand()
        
        cycle_idx = self.cycle % 4
        
        if cycle_idx == 0:
            # Motor A forward at speed 100
            msg.port = 0
            msg.command = SimpleMotorCommand.FORWARD
            msg.speed = 100
            self.get_logger().info('Sending: PORT_A FORWARD speed=100')
        elif cycle_idx == 1:
            # Motor B reverse at speed 80
            msg.port = 1
            msg.command = SimpleMotorCommand.REVERSE
            msg.speed = 80
            self.get_logger().info('Sending: PORT_B REVERSE speed=80')
        elif cycle_idx == 2:
            # Motor C forward at speed 60
            msg.port = 2
            msg.command = SimpleMotorCommand.FORWARD
            msg.speed = 60
            self.get_logger().info('Sending: PORT_C FORWARD speed=60')
        else:
            # All motors stop
            msg.port = 0
            msg.command = SimpleMotorCommand.STOP
            msg.speed = 0
            self.get_logger().info('Sending: ALL STOP')
        
        self.pub.publish(msg)
        self.cycle += 1


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
