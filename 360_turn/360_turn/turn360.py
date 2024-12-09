import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_twist)
        self.turning_speed = 0.1
        self.i = 0
        self.rad_turned = 0

    def publish_twist(self):
        if self.rad_turned < 2 * math.pi:
            print(f'Total rads turned: {self.rad_turned}')
            msg = Twist()
            self.rad_turned += self.turning_speed * self.timer_period
            msg.angular.z = self.turning_speed
            self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()