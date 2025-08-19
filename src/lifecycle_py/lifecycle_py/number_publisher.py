#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("In Constructor")
        self.number_ = 1
        self.publish_frequency_ = 1.0

    # Create ROS2 communications, connect to HW etc.
    def on_configure(self, state: LifecycleState):
        self.get_logger().info("In On Configure")
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(1.0 / self.publish_frequency_, self.publish_number)
        return TransitionCallbackReturn.SUCCESS

    # Destroy ROS2 communications, disconnect HW etc.
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("In On CleanUp")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
