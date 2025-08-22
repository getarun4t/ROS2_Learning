#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"

using Twist = geometry_msgs::msg::Twist;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace finale{

class TurtleBotController : public rclcpp_lifecycle::LifecycleNode 
{
public:
    TurtleBotController(const rclcpp::NodeOptions &options) : LifecycleNode("turtle_controller", options) 
    {
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);      
        server_activated_ =false;
        RCLCPP_INFO(get_logger(), "Action has been started");
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "In On Configure");
        
        cmd_vel_publisher_ = create_publisher<Twist>("/cmd_vel", 10);
        move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
            this,
            "move_turtle_",
            std::bind(&TurtleBotController::goal_callback, this, _1, _2),
            std::bind(&TurtleBotController::cancel_callback, this, _1),
            std::bind(&TurtleBotController::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );

        
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(get_logger(), "In On OnActivate");
        server_activated_=true;
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(get_logger(), "In On OnDeactivate");
        server_activated_=false;
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "In On Cleanup");
    
        cmd_vel_publisher_.reset();
        move_turtle_server_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "In On Shutdown");
        server_activated_=false;
        cmd_vel_publisher_.reset();
        move_turtle_server_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }   
 
private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp_action::Server<MoveTurtle>::SharedPtr move_turtle_server_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;
    std::mutex mutex_;

    bool server_activated_;

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveTurtle::Goal> goal){
            (void) uuid;
            RCLCPP_INFO(get_logger(), "Received a new goal");

            //Check if goal is activated
            if(!server_activated_){
                RCLCPP_WARN(get_logger(), "Action server not activated yet");
                return rclcpp_action::GoalResponse::REJECT;
            }

            //Goal policy
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if ((goal_handle_) && goal_handle_->is_active()){
                    RCLCPP_WARN(get_logger(), "Current goal still active");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }

            //Validate the new goal
            if (fabs(goal->linear_vel_x) > 3.0 ||
                fabs(goal->angular_vel_z) > 3.0 ||
                (goal->duration_sec <=0.0)){
                    RCLCPP_WARN(get_logger(), "Invalid goal");
                    return rclcpp_action::GoalResponse::REJECT;
                }

            RCLCPP_INFO(get_logger(), "Accepted a new goal");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveTurtleGoalHandle> goal_handle){
            (void) goal_handle;
            RCLCPP_INFO(get_logger(), "Received the cancelRequest");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

    void handle_accepted_callback(
        const std::shared_ptr<MoveTurtleGoalHandle> goal_handle){
            RCLCPP_INFO(get_logger(), "Executing the goal");
            execute_goal(goal_handle);
        }

    void execute_goal(
        const std::shared_ptr<MoveTurtleGoalHandle> goal_handle){
            {
                std::lock_guard<std::mutex> lock(mutex_);
                goal_handle_ = goal_handle;
            }

            double linear_vel_x = goal_handle->get_goal()->linear_vel_x;
            double angular_vel_z = goal_handle->get_goal()->angular_vel_z;
            int duration_sec = goal_handle->get_goal()->duration_sec;

            auto result = std::make_shared<MoveTurtle::Result>();
            auto start = now();
            auto duration = rclcpp::Duration(duration_sec, 0);

            rclcpp::Rate rate(10);

            while(rclcpp::ok()){
                if(now() - start > duration){
                    auto msg = Twist();
                    msg.linear.x = 0;
                    msg.angular.z = 0;
                    cmd_vel_publisher_->publish(msg);

                    result->success = true;
                    result-> message = "Success";
                    goal_handle->succeed(result);
                    return; 
                }

                if (!server_activated_){
                    auto msg = Twist();
                    msg.linear.x = 0;
                    msg.angular.z = 0;
                    cmd_vel_publisher_->publish(msg);
                    result->success = true;
                    result-> message = "Aborted";
                    goal_handle->abort(result);
                    return;
                }

                auto msg = Twist();
                msg.linear.x = linear_vel_x;
                msg.angular.z = angular_vel_z;
                rate.sleep();

                cmd_vel_publisher_->publish(msg);
            }
        }
};
}// namespace finale

/*
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<finale::TurtleBotController>(); 
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
*/

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(finale::TurtleBotController)