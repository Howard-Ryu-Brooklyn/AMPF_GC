# ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# msg 
from my_interfaces.msg import BoundingBox
from my_interfaces.msg import PolarCoor
from std_msgs.msg import Float32, Int16

# other library
from ultralytics import YOLO
import cv2
import pyzed.sl as sl
import math 
import numpy as np

# import time
class ZED_YOLO_PUBLISHER(Node):
    def __init__(self):
        super().__init__("zed_yolo_publisher")

        self.parameter_declare()

        self.pub_sub_timer_declare()
        
        self.print_info()
        
        self.initialization()
        
        
        # message declare
        self.bb_msg = BoundingBox()
        self.lost_msg = Int16()
        self.process_time = Float32()
        self.pcl_msg = PolarCoor()
        

    def close_publisher(self):
        self.get_logger().info("closing CV windows, ogl viewer and zed")
        
        # viewer.exit()
        cv2.destroyAllWindows()
        self.zed.close()

    def parameter_declare(self):
        # parameter
        self.timer_period = self.declare_parameter("timer_period", 0.04)  # [sec] 0.04 -> 25hz 
        self.ip_address = self.declare_parameter("ip_address", '192.168.1.110')
        self.resolution = self.declare_parameter("resolution", 'HD1080')
        self.model_path = self.declare_parameter("model_path", '/home/humble/ros2_ws/src/AMPF_UbuntuPC/sensing/yolov8/runs/detect/red_epoch1000/weights/best.pt')
        
        self.timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )
        
        self.ip_address = (
            self.get_parameter("ip_address").get_parameter_value().string_value
        )
        
        self.resolution = (
            self.get_parameter("resolution").get_parameter_value().string_value
        )
        
        self.model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        
    def pub_sub_timer_declare(self):
        
        self.bounding_box_pub_ = self.create_publisher(
            BoundingBox, "/boundingbox", qos_profile_sensor_data
        )

        self.process_time_pub_ = self.create_publisher(
            Float32, "/process_time", qos_profile_sensor_data
        )

        self.lost_leader_pub_ = self.create_publisher(
            Int16, "/lost_leader", qos_profile_sensor_data
        )
        
        self.polar_coordinate_pub_ = self.create_publisher(
            PolarCoor, "/polar_coordinate", qos_profile_sensor_data
        )

        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        
    def print_info(self):
        # print info
        self.get_logger().info("UWB publisher node has been initialized.")
        self.get_logger().info("parameter value is shown as below")
        self.get_logger().info(f"timer_period: {self.timer_period}")
        self.get_logger().info(f"ip_address: {self.ip_address}")
        self.get_logger().info(f"resolution: {self.resolution}")
        self.get_logger().info(f"yolo_model_path: {self.model_path}")
    
    def parse_args(self, init):
        
        if len(self.ip_address)>0 :
            ip_str = self.ip_address
            if ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4 and len(ip_str.split(':'))==2:
                init.set_from_stream(ip_str.split(':')[0],int(ip_str.split(':')[1]))
                self.get_logger().info(f"[Sample] Using Stream input, IP : {ip_str}")
            elif ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4:
                init.set_from_stream(ip_str)
                self.get_logger().info(f"[Sample] Using Stream input, IP : {ip_str}")
            else :
                self.get_logger().info(f"Unvalid IP format. Using live stream")
                
        if ("HD2K" in self.resolution):
            init.camera_resolution = sl.RESOLUTION.HD2K
            self.get_logger().info(f"[Sample] Using Camera in resolution HD2K")
        elif ("HD1200" in self.resolution):
            init.camera_resolution = sl.RESOLUTION.HD1200
            self.get_logger().info(f"[Sample] Using Camera in resolution HD1200")
        elif ("HD1080" in self.resolution):
            init.camera_resolution = sl.RESOLUTION.HD1080
            self.get_logger().info(f"[Sample] Using Camera in resolution HD1080")
        elif ("HD720" in self.resolution):
            init.camera_resolution = sl.RESOLUTION.HD720
            self.get_logger().info(f"[Sample] Using Camera in resolution HD720")
        elif ("SVGA" in self.resolution):
            init.camera_resolution = sl.RESOLUTION.SVGA
            self.get_logger().info(f"[Sample] Using Camera in resolution SVGA")
        elif ("VGA" in self.resolution):
            init.camera_resolution = sl.RESOLUTION.VGA
            self.get_logger().info(f"[Sample] Using Camera in resolution VGA")
        elif len(self.resolution)>0: 
            self.get_logger().info(f"[Sample] No valid resolution entered. Using default")
        else : 
            self.get_logger().info(f"[Sample] Using default resolution")

    def print_camera_information(self, cam):
        cam_info = cam.get_camera_information()
        self.get_logger().info("ZED Model                 : {0}".format(cam_info.camera_model))
        self.get_logger().info("ZED Serial Number         : {0}".format(cam_info.serial_number))
        self.get_logger().info("ZED Camera Firmware       : {0}/{1}".format(cam_info.camera_configuration.firmware_version,cam_info.sensors_configuration.firmware_version))
        self.get_logger().info("ZED Camera Resolution     : {0}x{1}".format(round(cam_info.camera_configuration.resolution.width, 2), cam.get_camera_information().camera_configuration.resolution.height))
        self.get_logger().info("ZED Camera FPS            : {0}".format(int(cam_info.camera_configuration.fps)))

    def initialization(self):
                
        self.get_logger().info("Running ZED initialization")

        init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.NEURAL,# NEURAL
                                    coordinate_units=sl.UNIT.METER,
                                    coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD) # coordinate frame define, same with ROS, y follows right handled rule
        self.parse_args(init)
        
        self.zed = sl.Camera()
        
        self.YOLO = YOLO(self.model_path)
        
        # check open
        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().info(repr(status))
            exit()

        # CV window
        self.win_name = "YOLOv8 Detection Viewer"
        self.mat = sl.Mat()
        cv2.namedWindow(self.win_name)
        self.print_camera_information(self.zed)
        
        res = sl.Resolution()
        res.width = self.zed.get_camera_information().camera_configuration.resolution.width
        res.height = self.zed.get_camera_information().camera_configuration.resolution.height
        # camera_model = self.zed.get_camera_information().camera_model
        
        # Create OpenGL viewer
        # viewer = gl.GLViewer()
        # viewer.init(1, sys.argv, camera_model, res)
        self.point_cloud = sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU) # F32_C4: float 32 with 4 channels
        
        # CV image plot variables
        self.blue_color = (98, 17, 0)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        # variables
        self.distance = -1.0
        self.angle_rad = 0
        self.dx = self.dy = self.dw = self.dh = 0
        self.trajectory = []

    def timer_callback(self):
        
        self.bb_msg.detected = False
        
        t_b = self.get_clock().now()
        
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS: #and viewer.is_available():
            # Retrieve data
            self.zed.retrieve_image(self.mat, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
            
            # processing image data
            cvImage = self.mat.get_data()
            frame_dim3 = cvImage.transpose(2, 0, 1)[0:3].transpose(1, 2, 0)
            
            # yolo model
            results = self.YOLO.track(frame_dim3, persist=True)
            annotated_frame = results[0].plot()
            boxes = results[0].boxes.xywh.cpu()
            
            if (results[0].boxes.id) != None:
                self.bb_msg.detected = True
                
                # Plot the tracks
                x, y, w, h = boxes[0]
                self.trajectory.append((float(x), float(y)))  # x, y center point
                if len(self.trajectory) > 30:  # retain 90 tracks for 90 frames
                    self.trajectory.pop(0)

                # Draw the tracking lines
                points = np.hstack(self.trajectory).astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
                
                self.dx=int(x.item())
                self.dy=int(y.item())
                self.dw=int(w.item())
                self.dh=int(h.item())
                
                # self.bb_msg.id = results[0].boxes.id
                # self.bb_msg.name = 
                self.bb_msg.xmin = self.dx - self.dw
                self.bb_msg.xmax = self.dx + self.dw
                self.bb_msg.ymin = self.dy - self.dh
                self.bb_msg.ymax = self.dy + self.dh
                # self.bb_msg.probability = 
                
            # Get self.distance from point cloud
            err, point_cloud_value = self.point_cloud.get_value(self.dx, self.dy)
            
            # point_cloud_value[0] = x (foward)
            # point_cloud_value[1] = y (right) -> atan(y/x)
            # point_cloud_value[2] = z (up)
            
            if math.isfinite(point_cloud_value[2]):
                self.distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
                self.angle_rad = math.atan(point_cloud_value[1]/point_cloud_value[0])
            
            self.pcl_msg.x = point_cloud_value[0]
            self.pcl_msg.y = point_cloud_value[1]
            self.pcl_msg.r = self.distance
            self.pcl_msg.theta = self.angle_rad
            
            distance_string = "(r,theta): {0} [m], {1} [deg]".format(round(self.distance, 2), round(self.angle_rad*180/math.pi,2))
            point_cloud_string = "(x,y,z): {0}, {1}, {2} [m]".format(round(point_cloud_value[0], 2), round(point_cloud_value[1], 2), round(point_cloud_value[2], 2))
            
            # Plot information 
            cv2.putText(annotated_frame, distance_string, (self.dx-3*self.dw, self.dy+self.dh), fontFace=self.font, fontScale=1, color=self.blue_color, thickness=2, lineType=cv2.LINE_AA)
            cv2.putText(annotated_frame, point_cloud_string, (self.dx-3*self.dw, self.dy+math.floor(2*self.dh)), fontFace=self.font, fontScale=1, color=self.blue_color, thickness=2, lineType=cv2.LINE_AA)
            cv2.circle(annotated_frame, (self.dx, self.dy), radius=5, color=self.blue_color, thickness=10)
            
            # viewers update
            # viewer.updateData(self.point_cloud)
            cv2.imshow(self.win_name, annotated_frame)
            cv2.waitKey(5) #[ms]
            
        t_a = self.get_clock().now()  # time after transformation
        self.process_time.data = (t_a.nanoseconds - t_b.nanoseconds) / 1e6
        
        self.lost_msg.data = not self.bb_msg.detected
        
        # publish
        self.bb_msg.header.stamp = self.get_clock().now().to_msg()
        self.bounding_box_pub_.publish(self.bb_msg)
        
        self.pcl_msg.header.stamp = self.get_clock().now().to_msg()
        self.polar_coordinate_pub_.publish(self.pcl_msg)
        
        self.lost_leader_pub_.publish(self.lost_msg)
        
        self.process_time_pub_.publish(self.process_time)
