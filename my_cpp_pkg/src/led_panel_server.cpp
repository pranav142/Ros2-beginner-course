#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led_status.hpp"
#include "my_robot_interfaces/msg/set_led_status.hpp"

class LedPanelServerNode : public rclcpp::Node
{
public:
    LedPanelServerNode() : Node("add_two_ints_server")
    {
        std::vector<bool> default_led_states = {false, false, false};
        this->declare_parameter<std::vector<bool>>("led_states", default_led_states);
        led_panel = this->get_parameter("led_states").as_bool_array();
        service_ = this->create_service<my_robot_interfaces::srv::SetLedStatus>("set_led", std::bind(&LedPanelServerNode::set_led_callback, this, std::placeholders::_1, std::placeholders::_2));

        publisher_ = this->create_publisher<my_robot_interfaces::msg::SetLedStatus>("led_panel_state", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LedPanelServerNode::publish_led_status, this));

        RCLCPP_INFO(this->get_logger(), "CPP Led Panel Server Node Started");
    }

private:
    void publish_led_status()
    {
        auto msg = my_robot_interfaces::msg::SetLedStatus();
        msg.led_0 = led_panel[0];
        msg.led_1 = led_panel[1];
        msg.led_2 = led_panel[2];
        publisher_->publish(msg);
    }

    void set_led_callback(my_robot_interfaces::srv::SetLedStatus::Request::SharedPtr request, my_robot_interfaces::srv::SetLedStatus::Response::SharedPtr response)
    {
        led_panel[request->led_number - 1] = request->state;
        response->success = true;
    }

    std::vector<bool> led_panel;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_robot_interfaces::msg::SetLedStatus>::SharedPtr publisher_;
    rclcpp::Service<my_robot_interfaces::srv::SetLedStatus>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}