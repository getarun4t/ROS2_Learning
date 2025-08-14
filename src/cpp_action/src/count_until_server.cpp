#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilServerNode : public rclcpp::Node 
{
public:
    CountUntilServerNode() : Node("count_until_server") 
    {
        count_until_server_ = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServerNode::goal_callback, this, _1, _2),
            std::bind(&CountUntilServerNode::cancel_callback, this, _1),
            std::bind(&CountUntilServerNode::handle_accepted_callback_callback, this, _1)
        );
        RCLCPP_INFO(get_logger(), "Action Server has been started");
    }
 
private:
    rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const CountUntil::Goal> goal){
            (void) uuid;
            RCLCPP_INFO(get_logger(), "Received a goal");
            if(goal->target_number <= 0.0){
                RCLCPP_WARN(get_logger(), "target number <= 0 \nRejecting the goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(get_logger(), "Accepting the goal");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<CountUntilGoalHandle> goal_handle){
            (void) goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }
    
    void handle_accepted_callback_callback(
        const std::shared_ptr<CountUntilGoalHandle> goal_handle){
            RCLCPP_INFO(get_logger(), "Executing the goal");
            execute_goal(goal_handle);
        }

    void execute_goal(
        const std::shared_ptr<CountUntilGoalHandle> goal_handle){
            // Get request from goal
            int target_number = goal_handle->get_goal()->target_number;
            double period = goal_handle->get_goal()->period;

            //Execute the goal
            int counter=0;
            auto feedback = std::make_shared<CountUntil::Feedback>();
            rclcpp::Rate loop_rate(1.0/period);
            for(int i=0; i<target_number; ++i){
                counter++;
                RCLCPP_INFO(get_logger(), "Current number is : %d", counter);
                feedback->current_number = counter;
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();
            }

            //Set final state and return result
            auto result = std::make_shared<CountUntil::Result>();
            result->reached_number = counter;
            goal_handle->succeed(result);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}