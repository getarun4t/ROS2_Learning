#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

using namespace std::chrono_literals;
 
class HardwareStatusPublisher : public rclcpp::Node 
{
public:
    HardwareStatusPublisher() : Node("hardware_status_publisher") 
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware__status", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&HardwareStatusPublisher::publishHardwareStatus, this));
        RCLCPP_INFO(this->get_logger(), "Hardware status publisher has been started");
    }
 
private:
    void publishHardwareStatus(){
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temparature = 3.7;
        msg.are_motors_ready = true;
        msg.debug_message = "Nothing special";
        publisher_->publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisher>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}