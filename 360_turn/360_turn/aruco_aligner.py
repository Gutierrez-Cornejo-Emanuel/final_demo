import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from alpha_delta.msg import AlphaDelta

class ArucoAligner(Node):

    def __init__(self):
        super().__init__('aruco_aligner')
        self.publisher = self.create_publisher(Float64, '/turn', 10)
        self.turn_complete_sub = self.create_subscription(Float64, '/turn_complete',self.on_turn_complete ,10)
        self.subscriber = self.create_subscription(AlphaDelta, '/aruco_pose',self.alpha_delta_callback ,10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.turn360)
        self.turning = False
        self.alpha_deltas = {}
    def turn360(self):
        msg = Float64()
        msg.data = 360.0
        self.publisher.publish(msg)
        self.timer.destroy()
    def alpha_delta_callback(self, msg: AlphaDelta):
        if msg.id in [10, 20, 30, 1]:
            if msg.id not in self.alpha_deltas:
                self.alpha_deltas[msg.id] = msg
            elif self.alpha_deltas[msg.id].alpha > msg.alpha:
                self.alpha_deltas[msg.id] = msg
        print(self.alpha_deltas)
    def on_turn_complete(self, msg: Float64):
        if msg.data == 360:
            print(self.alpha_deltas)

        





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