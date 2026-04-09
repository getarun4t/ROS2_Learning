#include "rclcpp/rclcpp.hpp"
 
class MyCustomNode : public rclcpp::Node //MODIFY_NAME
{
public:
    MyCustomNode() : Node("node_name") //MODIFY_NAME
    {
    }
 
private:
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); //MODIFY_NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}