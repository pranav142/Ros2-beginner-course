#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/turtles_alive.hpp"
#include <random> 

//Spawn Service 
// Kill Service
namespace turtlegame
{
    class Turtle
    {
    public:
        double x;
        double y;
        std::string turtle_name;

        Turtle(double x_val, double y_val, std::string turtle_name): x(x_val), y(y_val), turtle_name(turtle_name){}
    };
}

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner"), counter(0)
    {  
        publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtlesAlive>("alive_turtles", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&TurtleSpawnerNode::publish_alive_turtles, this));
        RCLCPP_INFO(this->get_logger(), "Turtle Spawner Node Started");
    }

private:
    double generate_random_value(double lowerbound = 0, double upperbound = 11)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> distribution(lowerbound, upperbound);

        double random_value = distribution(gen);
        return random_value;
    }

    turtlegame::Turtle generate_random_turtle()
    {
        std::stringstream turtle_name_ss;
        turtle_name_ss << "turtle_" << counter;
        counter ++;

        double x = generate_random_value(0, 11);
        double y = generate_random_value(0, 11);

        turtlegame::Turtle random_turtle(x, y, turtle_name_ss.str()); 
        return random_turtle;
    }

    void publish_alive_turtles()
    {
        turtlegame::Turtle random_turtle = generate_random_turtle();
        alive_turtles.push_back(random_turtle);

        auto turtles_alive_msg = my_robot_interfaces::msg::TurtlesAlive();

        for (const auto &turtle : alive_turtles)
        {
            my_robot_interfaces::msg::Turtle turtle_msg;
            turtle_msg.name = turtle.turtle_name;
            turtle_msg.x = turtle.x;
            turtle_msg.y = turtle.y;
            turtles_alive_msg.turtles_alive.push_back(turtle_msg);
        }

        publisher_->publish(turtles_alive_msg);
    }


    std::vector<turtlegame::Turtle> alive_turtles;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter;
    rclcpp::Publisher<my_robot_interfaces::msg::TurtlesAlive>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

