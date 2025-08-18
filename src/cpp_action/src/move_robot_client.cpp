#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_robot.hpp"
#include "example_interfaces/msg/empty.hpp"
 
using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ClientGoalHandle<MoveRobot>;
using namespace std::placeholders;
using Empty = example_interfaces::msg::Empty;

class MoveRobotClientNode : public rclcpp::Node 
{
public:
    MoveRobotClientNode() : Node("move_robot_client") 
    {
        move_robot_client_ = rclcpp_action::create_client<MoveRobot>(
            this, 
            "count_until");

        cancel_subscriber_ = create_subscription<Empty>(
            "cancel_move", 
            10, 
            std::bind(&MoveRobotClientNode::callback_cancel_move), 
            this, 
            _1);
    }

    void send_goal(int target_position, int velocity){
        //Wait for the action server
        move_robot_client_->wait_for_action_server();

        auto goal=MoveRobot::Goal();
        goal.position = target_position;
        goal.velocity = velocity;

        //Add the callbacks
        auto options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        options.result_callback = std::bind(&MoveRobotClientNode::resultCallBack, this, _1);
        options.goal_response_callback= std::bind(&MoveRobotClientNode::goalAcceptCallBack, this, _1);
        options.feedback_callback= std::bind(&MoveRobotClientNode::feedbackCallBack, this, _1, _2);

        //Send the goal
        RCLCPP_INFO(get_logger(), "Sending the goal");
        move_robot_client_->async_send_goal(goal, options);
        
        //Cancel goal test
        //timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&MoveRobotClientNode::timer_callback, this));
    }
 
private:
    rclcpp_action::Client<MoveRobot>::SharedPtr move_robot_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    MoveRobotGoalHandle::SharedPtr goal_handle_;
    rclcpp::Subscription<MoveRobot>::SharedPtr cancel_subscriber_;

    void timer_callback(){
        RCLCPP_INFO(get_logger(), "Cancel the goal");
        move_robot_client_->async_cancel_goal(goal_handle_);

        timer_->cancel();
    }

    //Callback to receive result once goal is done
    void resultCallBack(const MoveRobotGoalHandle::WrappedResult &result){
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(get_logger(), "Goal Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED){
            RCLCPP_ERROR(get_logger(), "Goal Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED){
            RCLCPP_WARN(get_logger(), "Goal Cancelled");
        }

        RCLCPP_INFO(get_logger(), "Position : %d", result.result->position);
        RCLCPP_INFO(get_logger(), "Message : %d", result.result->message);
    }

    //Callback to know if the goal is accepted or rejected
    void goalAcceptCallBack(const MoveRobotGoalHandle::SharedPtr &goal_handle){
        if (!goal_handle){
            RCLCPP_ERROR(get_logger(), "Goal was rejected");
        }
        else{
            goal_handle_ =  goal_handle;
            RCLCPP_INFO(get_logger(), "Goal was accepted");
        }
    }

    //Callback to receive the feedbacks
    void feedbackCallBack(const MoveRobotGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const MoveRobot::Feedback> feedback){
        (void) goal_handle;
        int current_position = feedback->current_position;
        RCLCPP_INFO(get_logger(), "Feedback : %d", current_position);
    }

    void callback_cancel_move(const Empty::SharedPtr msg){
        (void)msg;
        cancel_goal();
    }

    void cancel_goal(){
        if(goal_handle_){
            move_robot_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotClientNode>(); 
    node->send_goal(2, 2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}