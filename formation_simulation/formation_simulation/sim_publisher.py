import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from my_interfaces.msg import Uwb
from my_interfaces.msg import PolarCoor
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped
from webots_ros2_msgs.msg import FloatStamped
from std_msgs.msg import Int16
import math


class SimPublisher(Node):
    def __init__(self):
        super().__init__("sim_publisher")
        self.uwb_publisher = self.create_publisher(
            Uwb, "/f2/uwb", qos_profile_sensor_data
        )
        self.f1lp_publisher = self.create_publisher(
            PolarCoor, "/f1/polar_coordinate", qos_profile_sensor_data
        )
        self.f2lp_publisher = self.create_publisher(
            PolarCoor, "/f2/polar_coordinate", qos_profile_sensor_data
        )
        self.fp_publisher = self.create_publisher(
            PolarCoor, "/follower_polar", qos_profile_sensor_data
        )
        self.f1_lost_leader_publisher = self.create_publisher(
            Int16, "/f1/lost_leader", qos_profile_sensor_data
        )
        self.f2_lost_leader_publisher = self.create_publisher(
            Int16, "/f2/lost_leader", qos_profile_sensor_data
        )

        self.timer = self.create_timer(0.02, self.timer_callback) # 25hz same with zed_yolo_publisher

        self.cf_gps_subscriber = self.create_subscription(
            PointStamped, "/Crazyflie/gps", self.cf_gps_callback, 10
        )
        self.f1_gps_subscriber = self.create_subscription(
            PointStamped, "/f1/TurtleBot3Burgerf1/gps", self.f1_gps_callback, 10
        )
        self.f2_gps_subscriber = self.create_subscription(
            PointStamped, "/f2/TurtleBot3Burgerf2/gps", self.f2_gps_callback, 10
        )
        self.f1_compass_subscriber = self.create_subscription(
            Vector3Stamped,
            "/f1/TurtleBot3Burgerf1/compass/north_vector",
            self.f1_compass_callback,
            10,
        )
        self.f2_compass_subscriber = self.create_subscription(
            Vector3Stamped,
            "/f2/TurtleBot3Burgerf2/compass/north_vector",
            self.f2_compass_callback,
            10,
        )

        self.cf_gps_msg = PointStamped()
        self.f1_gps_msg = PointStamped()
        self.f2_gps_msg = PointStamped()
        self.f1_compass_msg = FloatStamped()
        self.f2_compass_msg = FloatStamped()
        self.uwb_msg = Uwb()
        self.f1lp_msg = PolarCoor()
        self.f2lp_msg = PolarCoor()
        self.fp_msg = PolarCoor()
        self.f1_lost_msg = Int16()
        self.f2_lost_msg = Int16()

        self.R2D = 180.0 / math.pi
        self.D2R = math.pi / 180.0
        self.FOV = 40 * self.D2R
        print("Sim publisher node has been initialized.")

    def cf_gps_callback(self, msg):
        self.cf_gps_msg.point.x = msg.point.x
        self.cf_gps_msg.point.y = msg.point.y
        # self.cf_gps_msg.z = msg.point.z

    def f1_gps_callback(self, msg):
        self.f1_gps_msg.point.x = msg.point.x
        self.f1_gps_msg.point.y = msg.point.y

    def f2_gps_callback(self, msg):
        self.f2_gps_msg.point.x = msg.point.x
        self.f2_gps_msg.point.y = msg.point.y

    def f1_compass_callback(self, msg):
        angle_rad = math.atan2(msg.vector.x, msg.vector.y)
        self.f1_compass_msg.data = angle_rad

    def f2_compass_callback(self, msg):
        angle_rad = math.atan2(msg.vector.x, msg.vector.y)
        self.f2_compass_msg.data = angle_rad

    def timer_callback(self):
        # f1 and leader polar
        self.f1lp_msg.x = self.cf_gps_msg.point.x - self.f1_gps_msg.point.x
        self.f1lp_msg.y = self.cf_gps_msg.point.y - self.f1_gps_msg.point.y
        self.f1lp_msg.r = (self.f1lp_msg.x**2.0 + self.f1lp_msg.y**2.0) ** 0.5
        self.f1lp_msg.los = math.atan2(self.f1lp_msg.y, self.f1lp_msg.x)

        # change angle range 0 to 2pi
        if self.f1lp_msg.los < 0:
            self.f1lp_msg.los = self.f1lp_msg.los + 2 * math.pi
        else:
            self.f1lp_msg.los = self.f1lp_msg.los

        if self.f1_compass_msg.data < 0:
            self.f1lp_msg.local_angle = self.f1_compass_msg.data + 2 * math.pi
        else:
            self.f1lp_msg.local_angle = self.f1_compass_msg.data

        self.f1lp_msg.theta = self.f1lp_msg.los - self.f1lp_msg.local_angle

        # f2 and leader polar
        self.f2lp_msg.x = self.cf_gps_msg.point.x - self.f2_gps_msg.point.x
        self.f2lp_msg.y = self.cf_gps_msg.point.y - self.f2_gps_msg.point.y
        self.f2lp_msg.r = (self.f2lp_msg.x**2.0 + self.f2lp_msg.y**2.0) ** 0.5
        self.f2lp_msg.los = math.atan2(self.f2lp_msg.y, self.f2lp_msg.x)

        # change angle range 0 to 2pi
        if self.f2lp_msg.los < 0:
            self.f2lp_msg.los = self.f2lp_msg.los + 2 * math.pi
        else:
            self.f2lp_msg.los = self.f2lp_msg.los

        if self.f2_compass_msg.data < 0:
            self.f2lp_msg.local_angle = self.f2_compass_msg.data + 2 * math.pi
        else:
            self.f2lp_msg.local_angle = self.f2_compass_msg.data

        self.f2lp_msg.theta = self.f2lp_msg.los - self.f2lp_msg.local_angle

        # follower polar
        self.fp_msg.x = self.f1_gps_msg.point.x - self.f2_gps_msg.point.x
        self.fp_msg.y = self.f1_gps_msg.point.y - self.f2_gps_msg.point.y
        self.fp_msg.r = (self.fp_msg.x**2.0 + self.fp_msg.y**2.0) ** 0.5
        self.fp_msg.los = math.atan2(self.fp_msg.y, self.fp_msg.x)

        # change angle range 0 to 2pi
        if self.fp_msg.los < 0:
            self.fp_msg.los = self.fp_msg.los + 2 * math.pi
        else:
            self.fp_msg.los = self.fp_msg.los

        if self.f2_compass_msg.data < 0:
            self.fp_msg.local_angle = self.f2_compass_msg.data + 2 * math.pi
        else:
            self.fp_msg.local_angle = self.f2_compass_msg.data

        self.fp_msg.theta = self.fp_msg.los - self.fp_msg.local_angle

        # uwb msg
        self.uwb_msg.range = (
            ((self.f1_gps_msg.point.x - self.f2_gps_msg.point.x) ** 2)
            + ((self.f1_gps_msg.point.y - self.f2_gps_msg.point.y) ** 2)
        ) ** 0.5 *1000 #[mm]

        # f1 lost leader
        if self.f1lp_msg.theta > self.FOV:
            self.f1_lost_msg.data = 1
        elif self.f1lp_msg.theta < -self.FOV:
            self.f1_lost_msg.data = 1
        else:
            self.f1_lost_msg.data = 0

        # f2 lost leader
        if self.f2lp_msg.theta > self.FOV:
            self.f2_lost_msg.data = 1
        elif self.f2lp_msg.theta < -self.FOV:
            self.f2_lost_msg.data = 1
        else:
            self.f2_lost_msg.data = 0

        # publish msgs
        self.uwb_msg.header.stamp = self.get_clock().now().to_msg()
        self.uwb_publisher.publish(self.uwb_msg)

        self.f1lp_msg.header.stamp = self.get_clock().now().to_msg()
        self.f1lp_publisher.publish(self.f1lp_msg)

        self.f2lp_msg.header.stamp = self.get_clock().now().to_msg()
        self.f2lp_publisher.publish(self.f2lp_msg)

        self.fp_msg.header.stamp = self.get_clock().now().to_msg()
        self.fp_publisher.publish(self.fp_msg)

        self.f1_lost_leader_publisher.publish(self.f1_lost_msg)
        self.f2_lost_leader_publisher.publish(self.f2_lost_msg)


def main(args=None):
    rclpy.init(args=args)

    uwb_data_pub = SimPublisher()

    rclpy.spin(uwb_data_pub)

    uwb_data_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
