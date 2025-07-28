#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
 
using namespace std::placeholders;

class SmartPhoneNode : public rclcpp::Node 
{
public:
    SmartPhoneNode() : Node("smartphone") 
    {
        subscriber = create_subscription<example_interfaces::msg::String>("robot_news", 10, std::bind(&SmartPhoneNode::callbackRobotNews, this, _1));
        RCLCPP_INFO(get_logger(), "Smartphone has been started");
    }
 
private:
    void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg){
        RCLCPP_INFO(get_logger(), "%s", msg->data.c_str());
    }
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartPhoneNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}