import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class UIController(Node):
    def __init__(self):
        super().__init__('ui_node')
        self.pub_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.get_logger().info("UI Node Started.")

    def run_interface(self):
        time.sleep(1)
        
        while rclpy.ok():
            try:
                print("\n" + "="*30)
                print(" TURTLE CONTROLLER UI ")
                print("="*30)
                
                target = input(">> Select robot (1 for turtle1, 2 for turtle2): ").strip()
                if target not in ['1', '2']:
                    print("Invalid selection.")
                    continue

                try:
                    lin_x = float(input(">> Linear Velocity (x): "))
                    ang_z = float(input(">> Angular Velocity (z): "))
                except ValueError:
                    print("Please enter valid numbers.")
                    continue

                cmd = Twist()
                cmd.linear.x = lin_x
                cmd.angular.z = ang_z

                print(f"Sending commands for 1 second...")

         
                # We publish continuously for 1 second to override the Safety Node
                # allowing to back away from walls.
                end_time = time.time() + 1.0
                while time.time() < end_time:
                    if target == '1':
                        self.pub_turtle1.publish(cmd)
                    else:
                        self.pub_turtle2.publish(cmd)
                    time.sleep(0.1) # Send command every 0.1 seconds
                # --------------------------

                # Stop
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
