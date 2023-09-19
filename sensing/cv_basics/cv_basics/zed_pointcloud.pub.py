# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import PointCloud2  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from my_interfaces.msg import BoundingBox


class PointCloudSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_subscriber")

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            "/zed2i/zed_node/point_cloud/cloud_registered",
            self.pointcloud_callback,
            10,
        )
        self.pointcloud_sub  # prevent unused variable warning

        self.boundingbox_sub = self.create_subscription(
            BoundingBox, "/my_yolo/BoundingBox", self.boundingbox_callback, 10
        )
        self.boundingbox_sub  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Declare boundingbox message
        self.boundingbox = BoundingBox()

        self.get_logger().info("Initializing zed pointcloud publisher completed")

    def boundingbox_callback(self, boundingbox):
        """
        Callback function.
        """
        # Save data
        self.boundingbox = boundingbox

    def pointcloud_callback(self, pointcloud_data):
        """
        Callback function.
        """
        # pc2data=pc2.read_points(pointcloud_data, skip_nans=True, field_names=("x","y","z"))

        # print("pc2data x:{}, y:{}, z:{}".format(pc2data))

        # get frame
        if self.boundingbox.detected == 1:
            xmin = self.boundingbox.xmin
            xmax = self.boundingbox.xmax
            ymin = self.boundingbox.ymin
            ymax = self.boundingbox.ymax


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    pointcloud_subscriber = PointCloudSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(pointcloud_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pointcloud_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
