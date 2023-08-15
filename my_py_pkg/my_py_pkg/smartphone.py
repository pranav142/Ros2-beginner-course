import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class SmartphoneNode(Node):
    def __init__(self) -> None: 
        super().__init__("smartphone")
        self.get_logger().info("Created Smartphone Node")
        self.subscriber_ = self.create_subscription(String, "robot_news", self.robot_news_proccessing, 10)
    
    def robot_news_proccessing(self, msg: String) -> None: 
        self.get_logger().info(msg.data)
    

def main() -> None:
    rclpy.init()
    node = SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()