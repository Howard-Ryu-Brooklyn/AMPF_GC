# ros2 libaray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

# ros2 msgs
from sensor_msgs.msg import Image
from my_interfaces.msg import BoundingBox

# other library
from ultralytics import YOLO
import cv2

# Implementation Summary
# 1. image subscriber
# 2. bouding box publisher


class BB_publisher(Node):
    def __init__(self):
        # Node 클래스를 상속받았기 때문에 부모 클래스의 init에 정의된 것들을 그대로 받아오기 위해 super.__init__을 수행함
        # 입력한 인자는 node의 이름
        super().__init__("bb_publisher")

        # Declare parameters
        self.declare_parameter("img_show", True)
        self.declare_parameter("timer_period", 0.04)  # [sec] 25hz
        self.declare_parameter(
            "model_path",
            "/home/dcasl-icryu/ros2_ws/src/AMPF_UbuntuPC/yolov8/runs/detect/train/weights/best.pt",
        )

        # Get param values
        self.img_show = self.get_parameter("img_show").get_parameter_value().bool_value
        timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )
        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        # Node 클래스를 상속 받았고 이를 init을 해주었기 때문에 부모 클래스에 속한 publisher라는 맴버변수를 사용할 수 있는 것.
        self.publisher_ = self.create_publisher(
            BoundingBox, "/boundingbox", qos_profile_sensor_data
        )

        self.timer_ = self.create_timer(timer_period, self.pub_boundingbox_cb)

        self.subscription_ = self.create_subscription(
            Image, "/image", self.read_image_cb, qos_profile_sensor_data
        )

        # for convert opencv msg 2 ros image msg
        self.br = CvBridge()
        # load trained model to detect crazyflie
        self.objdetection = YOLO(model_path)
        self.results = []

        self.get_logger().info("Bounding Box Publisher Initialized")
        self.get_logger().info("Parameters are shown as below")
        self.get_logger().info(f"img_show: {self.img_show}")
        self.get_logger().info(f"timer_period: {timer_period}")
        self.get_logger().info(f"model_path: {model_path}")

    def pub_boundingbox_cb(self):
        bb_msg = BoundingBox()
        bb_msg.header.stamp = self.get_clock().now().to_msg()
        bb_msg.detected = False

        if len(self.results) != 0:
            # store bounding box information
            boxes = self.results[0].boxes
            cls = boxes.cls.cpu().detach().numpy()  # class
            conf = boxes.conf.cpu().detach().numpy()  # confidence
            boxpos = self.results[0].boxes.xyxy.cpu().detach().numpy()  # location

            if boxpos.size != 0:  # if something detected
                bb_msg.detected = True

                bb_msg.xmin = int(boxpos[0][0])
                bb_msg.ymin = int(boxpos[0][1])
                bb_msg.xmax = int(boxpos[0][2])
                bb_msg.ymax = int(boxpos[0][3])

                bb_msg.probability = float(conf[0])

                bb_msg.id = int(cls[0])
                bb_msg.name = self.results[0].names[0]

        self.publisher_.publish(bb_msg)

    def read_image_cb(self, img_msg):
        t_b = self.get_clock().now()  # time after transformation

        cur_frame = self.br.imgmsg_to_cv2(img_msg)
        frame_dim3 = cur_frame.transpose(2, 0, 1)[0:3].transpose(1, 2, 0)

        self.results = self.objdetection(frame_dim3)
        t_a = self.get_clock().now()  # time after transformation
        # print(f"total obj detection time: {(t_a.nanoseconds - t_b.nanoseconds)/1e6} ms")

        if self.img_show:
            annotated_frame = self.results[0].plot()
            cv2.imshow("YOLOv8 Inference", annotated_frame)
            cv2.waitKey(1)
