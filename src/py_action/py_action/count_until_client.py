#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import CountUntil
 
class CountUntilClientNode(Node): 
    def __init__(self):
        super().__init__("count_until_client") 
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")

    def send_goal(self, target_number, period):
        self.count_until_client_.wait_for_server()

        # Create a goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # Send the goal
        self.get_logger().info("Sending the goal")
        self.count_until_client_.send_goal_async(goal).add_done_callback(self.sendGoal_callback)

    def sendGoal_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal accepted by Server")
            self.goal_handle_.get_result_async().add_done_callback(self.goalResult_callback)
        else:
            self.get_logger().error("Goal rejected by Server")

    def goalResult_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Aborted")
        self.get_logger().info("Result: " + str(result.reached_number))

    
 
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode() 
    node.send_goal(6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()