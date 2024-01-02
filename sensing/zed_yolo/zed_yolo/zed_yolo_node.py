import rclpy
from zed_yolo.zed_yolo_publisher import ZED_YOLO_PUBLISHER


def main(args=None):
    rclpy.init(args=args)

    zed_yolo_publisher = ZED_YOLO_PUBLISHER()

    rclpy.spin(zed_yolo_publisher)

    zed_yolo_publisher.close_publisher()
    
    zed_yolo_publisher.destroy_node()
    rclpy.shutdown()
