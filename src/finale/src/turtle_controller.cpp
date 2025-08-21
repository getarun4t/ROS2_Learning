#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"

using Spawn = turtlesim::srv::Spawn;
using Kill = turtlesim::srv::Kill;
using Twist = geometry_msgs::msg::Twist;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;

using namespace std::placeholders;
using namespace std::chrono_literals;

class TurtleController : public rclcpp::Node 
{
public:
    TurtleController() : Node("turtle_controller") 
    {
        this->declare_parameter("turtle_name", "aama");
        turtle_name_ = this->get_parameter("turtle_name").as_string();

        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        spawn_turtle_client_ = this->create_client<Spawn>(
            "/spawn", rclcpp::ServicesQoS(), cb_group_);

        kill_turtle_client_ = this->create_client<Kill>(
            "/kill", rclcpp::ServicesQoS(), cb_group_);

        cmd_vel_publisher_ = create_publisher<Twist>("/" + turtle_name_ + "/cmd_vel", 10);

        spawn_turtle_thread_ = std::thread(std::bind(&TurtleController::spawn_turtle, this));

        move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
            this,
            "move_turtle_" + turtle_name_,
            std::bind(&TurtleController::goal_callback, this, _1, _2),
            std::bind(&TurtleController::cancel_callback, this, _1),
            std::bind(&TurtleController::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );

        RCLCPP_INFO(get_logger(), "Action has been started");
    }
 
private:
    std::string turtle_name_;

    rclcpp::Client<Spawn>::SharedPtr spawn_turtle_client_;
    rclcpp::Client<Kill>::SharedPtr kill_turtle_client_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp_action::Server<MoveTurtle>::SharedPtr move_turtle_server_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;
    std::mutex mutex_;

    std::thread spawn_turtle_thread_;
    std::thread kill_turtle_thread_;

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveTurtle::Goal> goal){
            (void) uuid;
            RCLCPP_INFO(get_logger(), "Received a new goal");

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

                auto msg = Twist();
                msg.linear.x = linear_vel_x;
                msg.angular.z = angular_vel_z;
                rate.sleep();

                cmd_vel_publisher_->publish(msg);
            }
        }

    void spawn_turtle(){
        spawn_turtle_client_->wait_for_service();

        auto request = std::make_shared<Spawn::Request>();
        request->name = turtle_name_;
        request->x = 5.0;
        request->y = 5.0;

        RCLCPP_INFO(get_logger(), "Trying to Spawn turtle");

        auto result = spawn_turtle_client_->async_send_request(request);

        RCLCPP_INFO(get_logger(), "Spawned turle : %s", result.get()->name.c_str());
    }

    void kill_turtle(){
        kill_turtle_client_->wait_for_service();

        auto request = std::make_shared<Kill::Request>();
        request->name = turtle_name_;
        RCLCPP_INFO(get_logger(), "Trying to Kill turtle");

        kill_turtle_client_->async_send_request(request);

        RCLCPP_INFO(get_logger(), "Turtle has been removed");
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>(); 
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}