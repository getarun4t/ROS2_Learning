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
        turtle_name_ = get_parameter("turle_name").as_string();

        spawn_turtle_client_ = create_client<Spawn>(
            "/spawn"
        );
        kill_turtle_client_ = create_client<Kill>(
            "/kill"
        );
    }
 
private:
    std::string turtle_name_;

    rclcpp::Client<Spawn>::SharedPtr spawn_turtle_client_;
    rclcpp::Client<Kill>::SharedPtr kill_turtle_client_;

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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}