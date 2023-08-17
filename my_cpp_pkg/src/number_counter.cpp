#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter(0)
    {
        this->declare_parameter<int>("test123", 0);
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&NumberCounterNode::number_processing, this, std::placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        service_ = this->create_service<example_interfaces::srv::SetBool>("reset_number_count", std::bind(&NumberCounterNode::reset_number_count_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "NumberCounter Node Created");
    }

private:
    void reset_number_count_callback(example_interfaces::srv::SetBool::Request::SharedPtr request, example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            counter = 0;
            response->message = "reset counter";
        }
        else
            response->message = "didn't reset timer";
        response->success = true;
    }
    void number_processing(example_interfaces::msg::Int64::SharedPtr msg)
    {
        auto number = msg->data;
        auto new_number = example_interfaces::msg::Int64();
        new_number.data = number + counter;
        publisher_->publish(new_number);
        counter += 2;
    }

    int counter;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}