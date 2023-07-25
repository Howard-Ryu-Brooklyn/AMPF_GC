from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from my_interfaces.msg import Uwb
import serial


# import time
class UWBPublisher(Node):
    def __init__(self):
        super().__init__("uwb_publisher")
        self.timer_period = self.declare_parameter("timer_period", 0.02)  # [sec] 50hz
        timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )

        self.publisher_ = self.create_publisher(Uwb, "/my_uwb", qos_profile_sensor_data)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.serial = serial.Serial(
            port="/dev/ttyACM0",  # COM is on windows, linux is different
            baudrate=115200,  # many different baudrates are available
            parity="N",
            stopbits=1,
            bytesize=8,
            timeout=1,  # 8 seconds seems to be a good timeout, may need to be increased
        )

        print("UWB publisher node has been initialized.")
        print("parameter value is shown as below")
        print(f"timer_period: {timer_period}")

        if self.serial.is_open:
            print("serial_name", self.serial.name)

    def timer_callback(self):
        msg = Uwb()

        # 한줄 읽기 \r\n 같은걸 인식하는듯?
        line = self.serial.readline()

        # str으로 변환하고 쉼표 제거
        line_str = str(line, "utf-8").replace(",", "")

        # 데이터에 단위가 섞여 있으므로 그걸 찾아서 분리
        ms_idx = line_str.find("ms")
        mm_idx = line_str.find("mm")
        dBm_idx = line_str.find("dBm")

        # custom msg에 값 할당
        if ms_idx > 0 and mm_idx > 0 and dBm_idx > 0:
            # 가끔 설명 글이 들어오거나 range detect못했을때 msg가 나옴 그때는 ms_idx가 -1
            msg.time_stamp = float(line_str[0 : ms_idx - 1])
            msg.range = float(line_str[ms_idx + 2 : mm_idx - 1])
            msg.rss = float(line_str[mm_idx + 2 : dBm_idx - 1])

        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
