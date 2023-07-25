# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from ultralytics import YOLO
import torch
from my_interfaces.msg import BoundingBox


class My_YOLO_Wrapper(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("my_yolo_wrapper")

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, "/zed2i/zed_node/rgb/image_rect_color", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        # Create the publisher. This publisher will publish bounding box information from yolo
        self.result_pub = self.create_publisher(BoundingBox, "my_yolo/BoundingBox", 10)
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        # Declare YOLO model with specific weight
        self.yolov8_model = YOLO(
            "/home/dcasl-icryu/ros2_ws/src/AMPF_UbuntuPC/yolov8/runs/detect/train/weights/best.pt"
        )
        # self.yolov8_model = YOLO('yolov8n.pt')

        self.get_logger().info("My_Yolo_wrapper has been Initialized")

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Declare boudning box to store the data to be published
        bb_msg = BoundingBox()

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        # (512,892,4)->(512,892,3)
        frame_dim3 = current_frame.transpose(2, 0, 1)[0:3].transpose(1, 2, 0)

        # YOLO
        results = self.yolov8_model(frame_dim3)

        # store bounding box information
        boxes = results[0].boxes
        cls = boxes.cls.cpu().detach().numpy()  # class
        conf = boxes.conf.cpu().detach().numpy()  # confidence
        boxpos = results[0].boxes.xyxy.cpu().detach().numpy()  # location

        if boxpos.size != 0:  # if something detected
            bb_msg.header.stamp = self.get_clock().now()

            bb_msg.xmin = int(boxpos[0][0])
            bb_msg.ymin = int(boxpos[0][1])
            bb_msg.xmax = int(boxpos[0][2])
            bb_msg.ymax = int(boxpos[0][3])

            bb_msg.probability = float(conf[0])

            bb_msg.id = int(cls[0])
            bb_msg.name = results[0].names[0]
            bb_msg.detected = 1

        self.result_pub.publish(bb_msg)

        # Display the annotated frame
        annotated_frame = results[0].plot()
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    my_yolo_wrapper = My_YOLO_Wrapper()

    # Spin the node so the callback function is called.
    rclpy.spin(my_yolo_wrapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_yolo_wrapper.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
