import rclpy
from my_yolo_wrapper.boundingbox_publisher import BB_publisher


def main(args=None):
    rclpy.init(args=args)

    BoundingBox_Node = BB_publisher()

    rclpy.spin(BoundingBox_Node)
    rclpy.shutdown()
