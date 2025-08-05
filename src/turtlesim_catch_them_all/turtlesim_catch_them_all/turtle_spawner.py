#!/usr/bin/env python3
from functools import partial
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn 
import random
import math
 
class TurtleSpawnerNode(Node): 
    def __init__(self):
        super().__init__("turle_spawner") 
        self.turtle_name_prefix = "Turtle"
        self.turtle_counter = 0
        self.spawn_client_ = self.create_client(Spawn, "/spawn")
        self.spawn_turtle_timer_ = self.create_timer(2.0, self.spawn_new_turtle)

    def spawn_new_turtle(self):
        self.turtle_counter+=1
        name = self.turtle_name_prefix + str(self.turtle_counter)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2*math.pi)
        self.call_spawn_service(name, x, y, theta)

    def call_spawn_service(self, turtle_name, x, y, theta):
        while not self.spawn_client_.wait_for_service(0.1):
            self.get_logger().warn("Waiting for spawn service... ")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.spawn_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, request=request))

    def callback_call_spawn_service(self, future, request):
        response: Spawn.Response = future.result()
        if response.name != "":
            self.get_logger().info("New alive turtle: "+ response.name)

        
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()