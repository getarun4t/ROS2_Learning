#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
    void timerCallBack(){
        RCLCPP_INFO(get_logger(), "Hello : %d", counter_);
        counter_++;
    }
    rclcpp::TimerBase::SharedPtr timer_; 
    int counter_;  
public:
    MyNode() : Node("cpp_test"){
        RCLCPP_INFO(get_logger(), "Hello World");
        timer_ = create_wall_timer(std::chrono::seconds(1), 
                        std::bind(&MyNode::timerCallBack, this));
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}