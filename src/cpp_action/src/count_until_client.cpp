#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"
 
using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilClientNode : public rclcpp::Node 
{
public:
    CountUntilClientNode() : Node("count_until_client") 
    {
        count_until_client = rclcpp_action::create_client<CountUntil>(
            this, 
            "count_until");
    }

    void send_goal(int target_number, double period){
        //Wait for the action server
        count_until_client->wait_for_action_server();

        auto goal=CountUntil::Goal();
        goal.target_number = target_number;
        goal.period = period;

        //Add the callbacks
        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.result_callback = std::bind(&CountUntilClientNode::resultCallBack, this, _1);
        options.goal_response_callback= std::bind(&CountUntilClientNode::goalAcceptCallBack, this, _1);
        options.feedback_callback= std::bind(&CountUntilClientNode::feedbackCallBack, this, _1, _2);

        //Send the goal
        RCLCPP_INFO(get_logger(), "Sending the goal");
        count_until_client->async_send_goal(goal, options);
    }
 
private:
    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client;

    //Callback to receive result once goal is done
    void resultCallBack(const CountUntilGoalHandle::WrappedResult &result){
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(get_logger(), "Goal Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED){
            RCLCPP_ERROR(get_logger(), "Goal Aborted");
        }

        int reached_number = result.result->reached_number;
        RCLCPP_INFO(get_logger(), "Result : %d", reached_number);
    }

    //Callback to know if the goal is accepted or rejected
    void goalAcceptCallBack(const CountUntilGoalHandle::SharedPtr &goal_handle){
        if (!goal_handle){
            RCLCPP_ERROR(get_logger(), "Goal was rejected");
        }
        else{
            RCLCPP_INFO(get_logger(), "Goal was accepted");
        }
    }

    //Callback to receive the feedbacks
    void feedbackCallBack(const CountUntilGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const CountUntil::Feedback> feedback){
        (void) goal_handle;
        int number = feedback->current_number;
        RCLCPP_INFO(get_logger(), "Feedback : %d", number);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>(); 
    node->send_goal(6, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}