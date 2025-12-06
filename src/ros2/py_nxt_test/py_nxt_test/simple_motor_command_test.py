import rclpy

from rclpy.node import Node
from nxt_msgs.msg import SimpleMotorCommand
from nxt_msgs.msg import Ports


class SimpleMotorCommandTestNode(Node):
    def __init__(self):
        super().__init__('simple_motor_command_test')
        
        self.pub = self.create_publisher(
            SimpleMotorCommand,
            'nxt/simple_motor_cmd',
            10
        )
        
        self.timer = self.create_timer(2.0, self.publish_commands)
        self.cycle = 0
        
        self.get_logger().info('Simple motor command test node started')


    def publish_commands(self):
        msg = SimpleMotorCommand()
        
        cycle_idx = self.cycle % 4
        
        if cycle_idx == 0:
            # Motor A forward at speed 60
            msg.port = Ports.PORT_A
            msg.command = SimpleMotorCommand.FORWARD
            msg.speed = 60
            self.get_logger().info('Sending: PORT_A FORWARD speed=60')
            self.pub.publish(msg)
        elif cycle_idx == 1:
            # Motor B reverse at speed 30
            msg.port = Ports.PORT_B
            msg.command = SimpleMotorCommand.REVERSE
            msg.speed = 30
            self.get_logger().info('Sending: PORT_B REVERSE speed=30')
            self.pub.publish(msg)
        elif cycle_idx == 2:
            # Motor C forward at speed 60
            msg.port = Ports.PORT_C
            msg.command = SimpleMotorCommand.FORWARD
            msg.speed = 60
            self.get_logger().info('Sending: PORT_C FORWARD speed=60')
            self.pub.publish(msg)
        else:
            self.get_logger().info('Sending: STOP on all ports')
            for i in range(Ports.NUM_MOTOR_PORTS):
                msg.port = i
                msg.command = SimpleMotorCommand.STOP
                self.pub.publish(msg)
        
        self.cycle += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMotorCommandTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
