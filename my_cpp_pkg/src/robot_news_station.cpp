#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("robot_news_station")
    {
        this->declare_parameter<std::string>("robot_name", "default");
        std::string robot_name = this->get_parameter("robot_name").as_string();
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RobotNewsStationNode::publish_news, this));
        RCLCPP_INFO(this->get_logger(),  "Robot News Station %s Node CPP created", robot_name.c_str());
    }

private:
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_news()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = "Hello World CPP";
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}