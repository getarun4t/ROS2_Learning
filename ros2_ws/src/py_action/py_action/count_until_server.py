#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from my_robot_interfaces.action import CountUntil
 
 
class CountUntilServerNode(Node): 
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []

        self.count_until_server_ =  ActionServer(self, CountUntil, "count_until", goal_callback=self.goal_callback, cancel_callback=self.cancelGoal_callback, execute_callback=self.execute_callback, handle_accepted_callback=self.handleAccepted_callback, callback_group=ReentrantCallbackGroup())

        self.get_logger().info("Action Server has been started")

    # Used for validating the arguments of the request
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received a Goal")

        # Policy: Refuse new goal if current goal is active
        #with self.goal_lock_:
        #    if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #        self.get_logger().warn("A goal is already active, rejecting new goal")
        #        return GoalResponse.REJECT

        # Validate arguments of the call
        if goal_request.target_number <= 0:
            self.get_logger().warn("Rejecting the Goal")
            return GoalResponse.REJECT
        
        #Policy: Preempt existing goal when new goal received (TODO:pending fixes)
        #with self.goal_lock_:
        #    if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #        self.get_logger().warn("New goal received, aborting")
        #        self.goal_handle_.abort()

        self.get_logger().info("Accepting the Goal")
        return GoalResponse.ACCEPT

    def handleAccepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        # Get request from the goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if not self.goal_handle_.is_active:
                result.reached_number = counter
                self.get_logger().warn("Aborting the goal, returning: " + str(counter))
                self.process_next_goal_in_queue()
                return result

            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            
            counter+=1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
            
        
        # Once done, set goal final state
        goal_handle.succeed()

        # Send the result
        result.reached_number = counter
        self.process_next_goal_in_queue()
        return result
    
    def cancelGoal_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancelRequest")
        return CancelResponse.ACCEPT # Or reject as required
 
    def process_next_goal_in_queue(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute() 
            else:
                self.goal_handle_ = None

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()