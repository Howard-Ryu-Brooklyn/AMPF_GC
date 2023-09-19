import rclpy
from bounding_box.bounding_box_publisher import (
    BoundingBoxPublisher,
)


def main(args=None):
    rclpy.init(args=args)

    BoundingBoxNode = BoundingBoxPublisher()

    rclpy.spin(BoundingBoxNode)
    rclpy.shutdown()
