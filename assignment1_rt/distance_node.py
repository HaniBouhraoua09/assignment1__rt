import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        # Subscribers
        self.create_subscription(Pose, '/turtle1/pose', self.update_p1, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.update_p2, 10)

        # Publishers
        self.dist_pub = self.create_publisher(Float32, '/distance', 10)
        self.pub1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.p1 = None
        self.p2 = None

    def update_p1(self, msg):
        self.p1 = msg
        self.check()

    def update_p2(self, msg):
        self.p2 = msg
        self.check()

    def check(self):
        if self.p1 is None or self.p2 is None:
            return

        # 1. Distance
        dist = math.sqrt((self.p1.x - self.p2.x)**2 + (self.p1.y - self.p2.y)**2)
        
        msg = Float32()
        msg.data = dist
        self.dist_pub.publish(msg)

        # 2. Safety Logic
        stop = Twist() 
        
        # Collision check (Threshold 1.5)
        if dist < 1.5:
            self.get_logger().warn(f"Too Close! ({dist:.2f})")
            self.pub1.publish(stop)
            self.pub2.publish(stop)

        # Wall checks (Limits: 1.0 to 10.0)
        if self.p1.x < 1.0 or self.p1.x > 10.0 or self.p1.y < 1.0 or self.p1.y > 10.0:
            self.get_logger().warn("Turtle 1 hitting wall!")
            self.pub1.publish(stop)

        if self.p2.x < 1.0 or self.p2.x > 10.0 or self.p2.y < 1.0 or self.p2.y > 10.0:
            self.get_logger().warn("Turtle 2 hitting wall!")
            self.pub2.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
