########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    This sample demonstrates how to capture a live 3D point cloud   
    with the ZED SDK and display the result in an OpenGL window.    
"""

import sys
import ogl_viewer.viewer as gl

import pyzed.sl as sl
import argparse
import math 
import numpy as np

# YOLO
import cv2
from ultralytics import YOLO

def parse_args(init):
    if len(opt.input_svo_file)>0 and opt.input_svo_file.endswith(".svo"):
        init.set_from_svo_file(opt.input_svo_file)
        print("[Sample] Using SVO File input: {0}".format(opt.input_svo_file))
    elif len(opt.ip_address)>0 :
        ip_str = opt.ip_address
        if ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4 and len(ip_str.split(':'))==2:
            init.set_from_stream(ip_str.split(':')[0],int(ip_str.split(':')[1]))
            print("[Sample] Using Stream input, IP : ",ip_str)
        elif ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4:
            init.set_from_stream(ip_str)
            print("[Sample] Using Stream input, IP : ",ip_str)
        else :
            print("Unvalid IP format. Using live stream")
            
    if ("HD2K" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD2K
        print("[Sample] Using Camera in resolution HD2K")
    elif ("HD1200" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1200
        print("[Sample] Using Camera in resolution HD1200")
    elif ("HD1080" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1080
        print("[Sample] Using Camera in resolution HD1080")
    elif ("HD720" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD720
        print("[Sample] Using Camera in resolution HD720")
    elif ("SVGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.SVGA
        print("[Sample] Using Camera in resolution SVGA")
    elif ("VGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.VGA
        print("[Sample] Using Camera in resolution VGA")
    elif len(opt.resolution)>0: 
        print("[Sample] No valid resolution entered. Using default")
    else : 
        print("[Sample] Using default resolution")



def main():
    print("Running Depth Sensing sample ... Press 'Esc' to quit\nPress 's' to save the point cloud")

    init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.NEURAL,# NEURAL
                                 coordinate_units=sl.UNIT.METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD) # coordinate frame define, same with ROS, y follows right handled rule
    parse_args(init)
    
    zed = sl.Camera()
    
    # check open
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()
    
    # declare yolo model
    model = YOLO('/home/humble/ros2_ws/src/AMPF_UbuntuPC/sensing/yolov8/runs/detect/red_epoch1000/weights/best.pt')
    
    # CV window
    win_name = "YOLOv8 Detection Viewer"
    mat = sl.Mat()
    cv2.namedWindow(win_name)
    print_camera_information(zed)
    
    res = sl.Resolution()
    res.width = zed.get_camera_information().camera_configuration.resolution.width
    res.height = zed.get_camera_information().camera_configuration.resolution.height
    # camera_model = zed.get_camera_information().camera_model
    
    # Create OpenGL viewer
    # viewer = gl.GLViewer()
    # viewer.init(1, sys.argv, camera_model, res)
    point_cloud = sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU) # F32_C4: float 32 with 4 channels
    
    key = ''
    
    # CV image plot variables
    blue_color = (98, 17, 0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # variables
    distance = -1
    angle_rad = 0
    dx = dy = dw = dh = 0
    trajectory = []

    while key != 113:  # for 'q' key
        if zed.grab() == sl.ERROR_CODE.SUCCESS: #and viewer.is_available():
            # Retrieve data
            zed.retrieve_image(mat, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
            
            # processing image data
            cvImage = mat.get_data()
            frame_dim3 = cvImage.transpose(2, 0, 1)[0:3].transpose(1, 2, 0)
            
            # yolo model
            results = model.track(frame_dim3, persist=True)
            annotated_frame = results[0].plot()
            boxes = results[0].boxes.xywh.cpu()
                    
            if (results[0].boxes.id) != None:
                # Plot the tracks
                x, y, w, h = boxes[0]
                trajectory.append((float(x), float(y)))  # x, y center point
                if len(trajectory) > 30:  # retain 90 tracks for 90 frames
                    trajectory.pop(0)

                # Draw the tracking lines
                points = np.hstack(trajectory).astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
                
                dx=int(x.item())
                dy=int(y.item())
                dw=int(w.item())
                dh=int(h.item())
                
            # Get distance from point cloud
            err, point_cloud_value = point_cloud.get_value(dx, dy)
            
            # point_cloud_value[0] = x (foward)
            # point_cloud_value[1] = y (right) -> atan(y/x)
            # point_cloud_value[2] = z (up)
            
            if math.isfinite(point_cloud_value[2]):
                distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
                angle_rad = math.atan(point_cloud_value[1]/point_cloud_value[0])
            
            distance_string = "(r,theta): {0} [m], {1} [deg]".format(round(distance, 2), round(angle_rad*180/math.pi,2))
            point_cloud_string = "(x,y,z): {0}, {1}, {2} [m]".format(round(point_cloud_value[0], 2), round(point_cloud_value[1], 2), round(point_cloud_value[2], 2))
            
            # Plot information 
            cv2.putText(annotated_frame, distance_string, (dx-3*dw, dy+dh), fontFace=font, fontScale=1, color=blue_color, thickness=2, lineType=cv2.LINE_AA)
            cv2.putText(annotated_frame, point_cloud_string, (dx-3*dw, dy+math.floor(2*dh)), fontFace=font, fontScale=1, color=blue_color, thickness=2, lineType=cv2.LINE_AA)
            cv2.circle(annotated_frame, (dx, dy), radius=5, color=blue_color, thickness=10)
            
            # viewers update
            # viewer.updateData(point_cloud)
            cv2.imshow(win_name, annotated_frame)

        key = cv2.waitKey(5) #[ms]
    
    # viewer.exit()
    cv2.destroyAllWindows()
    zed.close()
    
    
def print_camera_information(cam):
    cam_info = cam.get_camera_information()
    print("ZED Model                 : {0}".format(cam_info.camera_model))
    print("ZED Serial Number         : {0}".format(cam_info.serial_number))
    print("ZED Camera Firmware       : {0}/{1}".format(cam_info.camera_configuration.firmware_version,cam_info.sensors_configuration.firmware_version))
    print("ZED Camera Resolution     : {0}x{1}".format(round(cam_info.camera_configuration.resolution.width, 2), cam.get_camera_information().camera_configuration.resolution.height))
    print("ZED Camera FPS            : {0}".format(int(cam_info.camera_configuration.fps)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_svo_file', type=str, help='Path to an .svo file, if you want to replay it',default = '')
    parser.add_argument('--ip_address', type=str, help='IP Adress, in format a.b.c.d:port or a.b.c.d, if you have a streaming setup', default = '')
    parser.add_argument('--resolution', type=str, help='Resolution, can be either HD2K, HD1200, HD1080, HD720, SVGA or VGA', default = '')
    opt = parser.parse_args()
    if len(opt.input_svo_file)>0 and len(opt.ip_address)>0:
        print("Specify only input_svo_file or ip_address, or none to use wired camera, not both. Exit program")
        exit()
    main() 
