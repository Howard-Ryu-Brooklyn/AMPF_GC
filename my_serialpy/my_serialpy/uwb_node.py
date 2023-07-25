import rclpy
from my_serialpy.uwb_publisher import UWBPublisher


def main(args=None):
    rclpy.init(args=args)

    uwb_data_pub = UWBPublisher()

    rclpy.spin(uwb_data_pub)

    uwb_data_pub.destroy_node()
    rclpy.shutdown()
