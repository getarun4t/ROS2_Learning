#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse
from my_robot_interfaces.action import MoveRobot
 
 
class MoveRobotServerNode(Node): 
    def __init__(self):
        super().__init__("move_robot_server") 
        self.robot_position_=50
        self.move_robot_server_= ActionServer(
            self,
            MoveRobot,
            "move_robot",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback
        )
        self.get_logger().info("Action Server has been started")
        self.get_logger().info("Robot position: "+ str(self.robot_position_))

    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Received a new goal")
        if(goal_request._position not in range (0,100)) or goal_request.velocity <=0:
            self.get_logger().error("Rejecting the goal: Invalid position or velocity")
            return GoalResponse.REJECT
        else:
            self.get_logger().info("Accepted the goal")
            return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        goal_position = goal_handle.request.position
        goal_velocity = goal_handle.request.velocity

        self.get_logger().info("Executing the goal")

        result = MoveRobot.Result()
        feedback = MoveRobot.Feedback()

        while rclpy.ok():
            diff = goal_position - self.robot_position_
            if diff == 0:
                result.position = self.robot_position_
                result.message = "Succeed"
                goal_handle.succeed()
                return result
            elif diff > 0:
                if diff>=goal_velocity:
                    self.robot_position_+=goal_velocity
                else:
                    self.robot_position_+=diff
            else:
                if abs(diff)>=goal_velocity:
                    self.robot_position_-=goal_velocity
                else:
                    self.robot_position_+=diff
            feedback.current_position = self.robot_position_
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("Robot position: "+ str(self.robot_position_))
            time.sleep(1.0)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()