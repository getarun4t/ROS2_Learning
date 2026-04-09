#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor

from components_py.node_1 import Node1
from components_py.node_2 import Node2



def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    node2 = Node2()
    executor = SingleThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()
    

if __name__ == "__main__":
    main()