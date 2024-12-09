import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from rclpy.qos import qos_profile_sensor_data
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback ,qos_profile_sensor_data)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_twist)
        self.turning_speed = 0.1
        self.i = 0
        self.deg_turned = 0
        self.previous_heading = -1
        self.threshold = 1
        self.turn_complete = False
    
    def odom_callback(self, msg: Odometry):
        if self.previous_heading == -1:
            self.previous_heading = msg._pose._pose.orientation.w
        self.deg_turned += abs(msg._pose._pose.orientation.w - self.previous_heading) * 360
        self.previous_heading = msg._pose._pose.orientation.w

        print(f"degrees turned {self.deg_turned}")

    def publish_twist(self):
        #print(f'Total rads turned: {self.rad_turned}')
        if self.deg_turned < 360:
            msg = Twist()
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