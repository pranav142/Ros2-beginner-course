#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::call_add_two_ints_server, this, 1, 4));
        RCLCPP_INFO(this->get_logger(), "Add Two Ints Client Created");
    }

private:
    void call_add_two_ints_server(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Error Connecting To Service");
        }       

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);

        try 
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, response->sum); 
        }  
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get result from service");
        }
    }

    std::thread thread1_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

