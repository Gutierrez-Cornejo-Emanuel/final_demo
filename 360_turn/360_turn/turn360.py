import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback ,10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_twist)
        self.turning_speed = 0.1
        self.i = 0
        self.rad_turned = 0
        self.initial_heading = -1
        self.current_heading = -1
        self.threshold = 1
    
    def odom_callback(self, msg: Odometry):
        if self.initial_heading == -1:
            self.initial_heading = msg._pose._pose.orientation.w
        self.current_heading = msg._pose._pose.orientation.w
        print(f"Initial: {self.initial_heading}, Current: {self.current_heading}")

    def publish_twist(self):
        print(f'Total rads turned: {self.rad_turned}')
        if abs(self.initial_heading - self.current_heading) > self.threshold:
            msg = Twist()
            self.rad_turned += self.turning_speed * self.timer_period
            msg.angular.z = self.turning_speed
            self.publisher_.publish(msg)
        else:
            self.timer.destroy()



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