import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from alpha_delta.msg import AlphaDelta
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class ArucoAligner(Node):

    def __init__(self):
        super().__init__('aruco_aligner')
        self.publisher = self.create_publisher(Float64, '/turn', 10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.turn_complete_sub = self.create_subscription(Float64, '/turn_complete',self.on_turn_complete ,10)
        self.subscriber = self.create_subscription(AlphaDelta, '/aruco_pose',self.alpha_delta_callback ,10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.turn360)
        self.turning = False
        self.alpha_deltas = {}


        self.distance_moved = 0
        self.distance_to_move = 0
        self.enabled_odom = False
        self.prev_x, self.prev_y = -1000, -1000 

        self.aligning_with_id = -1
    def turn360(self):
        msg = Float64()
        msg.data = 360.0
        self.publisher.publish(msg)
        self.timer.destroy()
    def alpha_delta_callback(self, msg: AlphaDelta):
        if self.aligning_with_id != -1:
            if abs(msg.alpha) > 3 and msg.id == self.aligning_with_id:
                twist = Twist()
                twist.angular.z = 0.005  * (1 if msg.alpha < 0 else -1)
                self.vel_publisher.publish(twist)
            elif abs(msg.alpha) < 3 and msg.id == self.aligning_with_id:
                self.aligning_with_id = -1
        if msg.id in [10, 20, 30, 1]:
            if msg.id not in self.alpha_deltas:
                self.alpha_deltas[msg.id] = msg
            elif abs(self.alpha_deltas[msg.id].alpha) > abs(msg.alpha):
                self.alpha_deltas[msg.id] = msg
        #print(self.alpha_deltas)

    def on_turn_complete(self, msg: Float64):
        if msg.data == 360:
            if 10 in self.alpha_deltas.keys():
                self.aligning_with_id = 10
                #self.distance_to_move = 50
                #self.distance_moved = 0
                #self.prev_x, self.prev_y = -1000, -1000
                #self.enabled_odom = True
                #self.odom_timer = self.create_timer(0.2, self.odom_driver)
    
    def odom_callback(self, msg: Odometry):
        if self.enabled_odom:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

            if self.prev_x == -1000:
                self.prev_x = x
                self.prev_y = y
            self.distance_moved += math.dist((x, y), (self.prev_x, self.prev_y))
            self.prev_x, self.prev_y = x,y
            self.get_logger().info('Total distance moved: "%s"' % str(self.distance_moved))
    
    def odom_driver(self):
        if self.distance_moved < self.distance_to_move:
            msg = Twist()
            msg.linear.x = 0.2
            self.publisher_.publish(msg)
        else:
            self.odom_timer.destroy()



        





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ArucoAligner()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()