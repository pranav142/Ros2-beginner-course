import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")

        self.get_logger().info("robot news station created")
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.create_timer(1, self.publish_data)

    def publish_data(self):
        msg = String()
        msg.data = "Hello World!"
        self.publisher_.publish(msg)
    
def main() -> None:
    rclpy.init()
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown() 


if __name__ == "__main__":
    main()
