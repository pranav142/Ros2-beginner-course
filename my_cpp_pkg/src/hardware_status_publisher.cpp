#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher")
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("robot_news", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HardwareStatusPublisherNode::publish_news, this));
        RCLCPP_INFO(this->get_logger(), "Hardware Status Publisher Node CPP created");
    }

private:
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_news()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 30; 
        msg.are_motors_ready = true;
        msg.debug_message = "all good from cpp";
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}