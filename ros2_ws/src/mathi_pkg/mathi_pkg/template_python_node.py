#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
 
class MyCustomNode(Node): #_MODIFY_NAME
    def __init__(self):
        super().__init__("node_name") #_MODIFY_NAME
 
 
def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() #_MODIFY_NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()