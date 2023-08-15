#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() 
        : Node("cpp_test"), 
        counter_(0), 
        logger_(this->get_logger())
    {
        RCLCPP_INFO(logger_, "hello cpp node OOP");

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timerCallback, this));
    }

private: 
    void timerCallback()
    {
        counter_++;
        RCLCPP_INFO(logger_, "Hello World! %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
    rclcpp::Logger logger_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}