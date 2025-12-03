import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class UIController(Node):
    def __init__(self):
        super().__init__('ui_node')
        # Create publishers for both turtles
        self.pub_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.get_logger().info("UI Node Started.")

    def run_interface(self):
        # Wait a bit for connections
        time.sleep(1)
        
        while rclpy.ok():
            try:
                print("\n" + "="*30)
                print(" TURTLE CONTROLLER UI ")
                print("="*30)
                
                # 1. Select the robot
                target = input(">> Select robot (1 for turtle1, 2 for turtle2): ").strip()
                
                if target not in ['1', '2']:
                    print("Invalid selection. Try again.")
                    continue

                # 2. Select the velocities
                try:
                    lin_x = float(input(">> Linear Velocity (x): "))
                    ang_z = float(input(">> Angular Velocity (z): "))
                except ValueError:
                    print("Please enter valid numbers.")
                    continue

                # Prepare message
                cmd = Twist()
                cmd.linear.x = lin_x
                cmd.angular.z = ang_z

                # 3. Publish and Move
                print("Moving for 1 second...")
                
                # Send commands continuously for 1 second to override safety stop if needed
                end_time = time.time() + 1.0
                while time.time() < end_time:
                    if target == '1':
                        self.pub_turtle1.publish(cmd)
                    else:
                        self.pub_turtle2.publish(cmd)
                    time.sleep(0.1)

                # 4. Stop
                stop_cmd = Twist()
                if target == '1':
                    self.pub_turtle1.publish(stop_cmd)
                else:
                    self.pub_turtle2.publish(stop_cmd)
                
                print("Stopped.")

            except KeyboardInterrupt:
                break

def main(args=None):
    rclpy.init(args=args)
    node = UIController()
    node.run_interface()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
