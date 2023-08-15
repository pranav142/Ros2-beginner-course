#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led_status.hpp"

class BatteryClientNode : public rclcpp::Node
{
public:
    BatteryClientNode() : Node("battery_client")
    {
        thread1_ = std::thread(std::bind(&BatteryClientNode::call_set_led_server, this, false));
        RCLCPP_INFO(this->get_logger(), "Battery Client Created");
    }

private:
    void call_set_led_server(bool battery_charged)
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLedStatus>("set_led");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Error Connecting To Service");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLedStatus::Request>();
        request->led_number = 3;

        if (battery_charged)
        {
            request->state = false;
        }
        else
            request->state = true;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Led has been set");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get result from service");
        }
    }

    std::thread thread1_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}