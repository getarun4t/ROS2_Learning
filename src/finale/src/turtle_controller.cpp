#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

using Spawn = turtlesim::srv::Spawn;
using Kill = turtlesim::srv::Kill;

class TurtleController : public rclcpp::Node 
{
public:
    TurtleController() : Node("turtle_controller") 
    {
        declare_parameter(
            "turtle_name", 
            rclcpp::PARAMETER_STRING
        );
        turtle_name_ = get_parameter("turtle_name").as_string();

        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        spawn_turtle_client_ = create_client<Spawn>(
            "/spawn",
            rclcpp::ServicesQoS(),
            cb_group_
        );
        kill_turtle_client_ = create_client<Kill>(
            "/kill",
            rclcpp::ServicesQoS(),
            cb_group_
        );

        spawn_turtle_thread_ = std::thread(std::bind(&TurtleController::spawn_turtle, this));
    }
 
private:
    std::string turtle_name_;

    rclcpp::Client<Spawn>::SharedPtr spawn_turtle_client_;
    rclcpp::Client<Kill>::SharedPtr kill_turtle_client_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    std::thread spawn_turtle_thread_;
    std::thread kill_turtle_thread_;

    void spawn_turtle(){
        spawn_turtle_client_->wait_for_service();

        auto request = std::make_shared<Spawn::Request>();
        request->name = turtle_name_;
        request->x = 5.0;
        request->y = 5.0;

        RCLCPP_INFO(get_logger(), "Trying to Spawn turtle");

        auto result = spawn_turtle_client_->async_send_request(request);

        RCLCPP_INFO(get_logger(), "Spawned turle : %s", result.get()->name.c_str());

        std::this_thread::sleep_for(std::chrono::seconds(3));
        kill_turtle_thread_ = std::thread(std::bind(&TurtleController::kill_turtle, this));

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