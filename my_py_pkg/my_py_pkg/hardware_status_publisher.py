import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")

        self.get_logger().info("hardware status publisher node created")
        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.create_timer(1, self.publish_data)

    def publish_data(self):
        msg = HardwareStatus()
        msg.temperature = 30
        msg.are_motors_ready = True
        msg.debug_message = "ALL GOOD"
        self.publisher_.publish(msg)
    
def main() -> None:
    rclpy.init()
    node = HardwareStatusPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown() 


if __name__ == "__main__":
    main()
