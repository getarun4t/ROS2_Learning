#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


#include "my_robot_interfaces/action/move_robot.hpp"

using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;
using namespace std::placeholders;
 
class MoveRobotServerNode : public rclcpp::Node 
{
public:
    MoveRobotServerNode() : Node("move_robot_server") 
    {
        robot_position_ = 50;
        call_back_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_robot_server_ = rclcpp_action::create_server<MoveRobot>(
            this,
            "move_robot",
            std::bind(&MoveRobotServerNode::goal_callback, this, _1, _2),
            std::bind(&MoveRobotServerNode::cancel_callback, this, _1),
            std::bind(&MoveRobotServerNode::handle_accepted_callback_callback, this, _1),
            rcl_action_server_get_default_options(),
            call_back_group_
        );
        RCLCPP_INFO(get_logger(), "Action Server has been started");
    }
 
private:
    int robot_position_;
    std::mutex mutex_;
    std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
    
    rclcpp::CallbackGroup::SharedPtr call_back_group_;
    rclcpp_action::Server<MoveRobot>::SharedPtr move_robot_server_;
    rclcpp_action::GoalUUID preempted_goal_id;

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveRobot::Goal> goal){
            RCLCPP_INFO(get_logger(), "Received a goal");

            //Validate new goal
            if(goal->position < 0 or goal->position >100 ){
                RCLCPP_WARN(get_logger(), "Position is not in the range (0,100) \nRejecting the goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
            if(goal->velocity <= 0 ){
                RCLCPP_WARN(get_logger(), "Velcoity is less than 0 \nRejecting the goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            //Pre-empt any existing goals
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(goal_handle_){
                    if(goal_handle_->is_active()){
                        preempted_goal_id = goal_handle_->get_goal_id();
                    }
                }
            }

            RCLCPP_INFO(get_logger(), "Accepted the goal");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle){
            (void) goal_handle;
            RCLCPP_INFO(get_logger(), "Received the cancelRequest");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

    void handle_accepted_callback_callback(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle){
            RCLCPP_INFO(get_logger(), "Executing the goal");
            execute_goal(goal_handle);
        }

    void execute_goal(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle){
            
            {
                std::lock_guard<std::mutex> lock(mutex_);
                goal_handle_ = goal_handle;
            }

            //Get request from the goal
            int target_position = goal_handle->get_goal()->position;
            int velocity = goal_handle->get_goal()->velocity;

            RCLCPP_INFO(get_logger(), "Target robot position: %d", int(target_position));
            RCLCPP_INFO(get_logger(), "Velocity : %d", int(velocity));
            RCLCPP_INFO(get_logger(), "Current robot position: %d", int(robot_position_));

            auto result = std::make_shared<MoveRobot::Result>();
            auto feedback = std::make_shared<MoveRobot::Feedback>();

            rclcpp::Rate loop_rate(1.0);
            while(rclcpp::ok()){
                //Check if pre-empt required
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                        if(goal_handle->get_goal_id() == preempted_goal_id){
                            result->position = robot_position_;
                            result->message = "Pre-empted by another goal";
                            goal_handle->abort(result);
                            return;
                        }
                }

                //Check if cancelling
                if(goal_handle->is_canceling()){
                    result->position = robot_position_;
                    if(target_position==robot_position_){
                        result->message = "Succeed";
                        goal_handle->succeed(result);
                    }
                    else{
                        result->message = "Cancelled";
                        goal_handle->canceled(result);
                    }
                    return;
                }
                

                auto diff = target_position - robot_position_;

                if (diff == 0){
                    result->position = robot_position_;
                    result->message = "Succeed";
                    goal_handle->succeed(result);
                    return;
                }
                
                else if (diff > 0){
                    if (diff>=velocity)
                        robot_position_+=velocity;
                    else
                        robot_position_+=diff;
                }
                
                else{
                    if (abs(diff)>=velocity)
                        robot_position_-=velocity;
                    else
                        robot_position_+=diff;
                }

                loop_rate.sleep();
                feedback->current_position = robot_position_;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(get_logger(), "Current robot position: %d", int(robot_position_));
            }
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotServerNode>(); 
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}