import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class DistanceNode(Node):
    def __init__(self):
	super().__init__('distance_node')

	# --- PARAMETERS (No more magic numbers!) ---
	# 1. Declare the parameters with default values
	self.declare_parameter('wall_min', 1.0)
	self.declare_parameter('wall_max', 10.0)
	self.declare_parameter('dist_threshold', 1.5)

	# 2. Read the values (This allows them to be changed via Launch file)
	self.wall_min = self.get_parameter('wall_min').get_parameter_value().double_value
	self.wall_max = self.get_parameter('wall_max').get_parameter_value().double_value
	self.dist_threshold = self.get_parameter('dist_threshold').get_parameter_value().double_value

	self.get_logger().info(f"Safety Params: Wall=[{self.wall_min},{self.wall_max}], Dist={self.dist_threshold}")

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
        
        # Collision check
	if dist < self.dist_threshold:  # Uses parameter
		self.get_logger().warn(f"Too Close! ({dist:.2f})")
		self.pub1.publish(stop)
		self.pub2.publish(stop)

	# Limit 2: Turtle 1 hitting walls
	if self.p1.x < self.wall_min or self.p1.x > self.wall_max or \
		self.p1.y < self.wall_min or self.p1.y > self.wall_max:
		self.get_logger().warn("Turtle 1 hitting wall!")
		self.pub1.publish(stop)

	# Limit 3: Turtle 2 hitting walls
	if self.p2.x < self.wall_min or self.p2.x > self.wall_max or \
		self.p2.y < self.wall_min or self.p2.y > self.wall_max:
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
