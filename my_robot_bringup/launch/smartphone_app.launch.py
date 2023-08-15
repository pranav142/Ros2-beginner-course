from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    names = ["giskard", "bb8", "daneel", "jander", "c3po"]
    robot_station_nodes = []

    for name in names:
        robot_station_nodes.append(
            Node(
                package="my_py_pkg", executable="robot_news_station", name=f"robot_news_station_{name}"
            )
        )

    smartphone_node = Node(
        package="my_py_pkg", executable="smartphone"
    )

    for node in robot_station_nodes:
        ld.add_action(node)
    ld.add_action(smartphone_node)
    return ld
