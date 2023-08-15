#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class myNode(Node):
    def __init__(self):
        super().__init__(node_name="py_test")
        self.counter_ = 0
        self.logger = self.get_logger()
        self.logger.info("Node 1 Started !!!!!!!")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.logger.info(f"Hello World {self.counter_}")


def main(args=None):
    rclpy.init(args=args)
    node = myNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
