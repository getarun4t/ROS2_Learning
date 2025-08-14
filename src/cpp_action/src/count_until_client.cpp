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

        //Send the goal
        RCLCPP_INFO(get_logger(), "Sending the goal");
        count_until_client->async_send_goal(goal, options);
    }
 
private:
    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client;

    //Callback to receive result once goal is done
    void resultCallBack(const CountUntilGoalHandle::WrappedResult &result){
        int reached_number = result.result->reached_number;
        RCLCPP_INFO(get_logger(), "Result : %d", reached_number);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>(); 
    node->send_goal(6, 1.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}