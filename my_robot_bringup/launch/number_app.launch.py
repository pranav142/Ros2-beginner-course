from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_publisher_node = Node(
        package="my_cpp_pkg", 
        executable="number_publisher",
        name="publisher_1"
    )

    number_counter_node = Node(
        package="my_cpp_pkg",
        executable="number_counter",
        name="counter_1"
    )

    ld.add_action(number_counter_node)
    ld.add_action(number_publisher_node)
    return ld