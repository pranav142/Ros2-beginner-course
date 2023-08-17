#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>

namespace turtlegame
{
    class velocity
    {
    public:
        double x;
        double y;
        double z;

        velocity(double x_val, double y_val, double z_val) : x(x_val), y(y_val), z(z_val) {}
    };

    class turtle_transformation
    {
    public:
        velocity linear_velocity;
        velocity angular_velocity;

        turtle_transformation(velocity lineear_velocity_val, velocity angular_velocity_val) : linear_velocity(lineear_velocity_val), angular_velocity(angular_velocity_val) {}
    };

    class position
    {
    public:
        double x;
        double y;
        double theta;

        position() {}
        position(double x_val, double y_val, double theta_val) : x(x_val), y(y_val), theta(theta_val) {}
    };
}

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("number_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleControllerNode::position_subscriber_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "turtle controller node");
    }

private:
    void position_subscriber_callback(turtlesim::msg::Pose::SharedPtr msg)
    {
        current_position.x = msg->x;
        current_position.y = msg->y;
        current_position.theta = msg->theta;

        turtlegame::position enemy_position(10, 10, 0);
        turtlegame::turtle_transformation transform = calculate_turtle_movement(enemy_position, current_position);
        publish_velocity(transform.linear_velocity, transform.angular_velocity);
    }

    double calculate_vector_magnitude(double x_1, double y_1, double x_2, double y_2)
    {
        double magnitude = std::sqrt((x_2 - x_1) * (x_2 - x_1) + (y_2 - y_1) * (y_2 - y_1));
        RCLCPP_INFO(this->get_logger(), "Magnitude Calculated: %f", magnitude);
        return magnitude;
    }

    double calculate_vector_theta(double x_1, double y_1, double x_2, double y_2, double current_theta)
    {
        double delta_x = x_2 - x_1;
        double delta_y = y_2 - y_1;

        double theta = std::atan2(delta_y, delta_x) - current_theta;

        while (theta > M_PI)
        {
            theta -= 2 * M_PI;
        }
        while (theta < -M_PI)
        {
            theta += 2 * M_PI;
        }

        RCLCPP_INFO(this->get_logger(), "Calculated Theta: %f", theta);
        return theta;
    }

    turtlegame::turtle_transformation calculate_turtle_movement(turtlegame::position &enemy_position, turtlegame::position &current_position)
    {
        double linear_magnitude = calculate_vector_magnitude(current_position.x, current_position.y, enemy_position.x, enemy_position.y);
        double theta = calculate_vector_theta(current_position.x, current_position.y, enemy_position.x, enemy_position.y, current_position.theta);
        turtlegame::velocity linear_velocity(linear_magnitude, 0, 0);
        turtlegame::velocity angular_velocity(0, 0, theta);
        turtlegame::turtle_transformation result(linear_velocity, angular_velocity);
        return result;
    }

    void publish_velocity(turtlegame::velocity &linear_velocity, turtlegame::velocity &angular_velocity) const
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_velocity.x;
        msg.linear.y = linear_velocity.y;
        msg.linear.z = linear_velocity.z;
        msg.angular.x = angular_velocity.x;
        msg.angular.y = angular_velocity.y;
        msg.angular.z = angular_velocity.z;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    turtlegame::position current_position;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}