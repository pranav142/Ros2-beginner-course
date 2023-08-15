import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial


class AddTwoIntsClientNode(Node):
    def __init__(self) -> None:
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_server(6, 7)

    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger.error("Waiting For Conenction To Server")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_add_two_ints, a=a, b=b))

    def callback_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"{a} + {b} = {response.sum}")
        except Exception as e:
            self.get_logger().error(f"Error Getting Response {e}")


def main() -> None:
    rclpy.init()
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()
