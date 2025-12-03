import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
import sys
import random

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for services...')

    def setup_turtles(self):
        # 1. Reset Turtle 1
        req_kill = Kill.Request()
        req_kill.name = 'turtle1'
        future_kill = self.kill_client.call_async(req_kill)
        rclpy.spin_until_future_complete(self, future_kill)

        # 2. Spawn Turtle 1 (Center)
        req1 = Spawn.Request()
        req1.x = 5.5
        req1.y = 5.5
        req1.theta = 0.0
        req1.name = 'turtle1'
        future1 = self.spawn_client.call_async(req1)
        rclpy.spin_until_future_complete(self, future1)

        # 3. Spawn Turtle 2 (Random)
        req2 = Spawn.Request()
        req2.x = float(random.randint(2, 9)) 
        req2.y = float(random.randint(2, 9))
        req2.theta = 0.0
        req2.name = 'turtle2'
        future2 = self.spawn_client.call_async(req2)
        rclpy.spin_until_future_complete(self, future2)
        self.get_logger().info(f'Turtle 2 Spawned at ({req2.x}, {req2.y})')

def main(args=None):
    rclpy.init(args=args)
    spawner = TurtleSpawner()
    spawner.setup_turtles()
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
