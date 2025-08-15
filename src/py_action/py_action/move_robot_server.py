#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from my_robot_interfaces.action import MoveRobot
 
 
class MoveRobotServerNode(Node): 
    def __init__(self):
        super().__init__("move_robot_server") 
        self.robot_position_=50
        self.goal_handle_:ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.move_robot_server_= ActionServer(
            self,
            MoveRobot,
            "move_robot",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Action Server has been started")
        self.get_logger().info("Robot position: "+ str(self.robot_position_))

    def cancel_callback(self, goal_handle:CancelResponse):
        self.get_logger().info("Received the cancel request")
        return CancelResponse.ACCEPT

    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Received a new goal")
        if(goal_request._position not in range (0,100)) or goal_request.velocity <=0:
            self.get_logger().error("Rejecting the goal: Invalid position or velocity")
            return GoalResponse.REJECT
        
        #New goal is valid, abort previous goal and accept new goal
        if self.goal_handle_ is not None and self.goal_handle_.is_active:
            self.goal_handle_.abort()

        self.get_logger().info("Accepted the goal")
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        goal_position = goal_handle.request.position
        goal_velocity = goal_handle.request.velocity

        self.get_logger().info("Executing the goal")

        result = MoveRobot.Result()
        feedback = MoveRobot.Feedback()

        while rclpy.ok():
            #Return if goal is rejected
            if not goal_handle.is_active:
                result.position = self.robot_position_
                result.message = "Pre-empted by another goal"
                return result
            
            #Cancel if requested
            if goal_handle.is_cancel_requested:
                result.position = self.robot_position_
                if goal_position == self.robot_position_:
                    result.message = "Succeed"
                    goal_handle.succeed()
                    return result
                result.message = "Cancelled by client"
                goal_handle.canceled()
                return result

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
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()