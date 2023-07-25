import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Specify files like yaml, xacro, urdf in this format
# config_common = os.path.join(
#     get_package_share_directory("pkg_name"), "folder", "file.yaml"
# )

# zed
# yolo
# leader dist ang
# ampf_gb_f1


def generate_launch_description():
    # declare and get launch argument
    # get Launch Argument, this should be matched to Declared Launch Argument
    # you do this because you want to config node or launch using launch argument
    # arg1 = LaunchConfiguration("arg1")

    # dc_arg1 = (
    #     DeclareLaunchArgument(
    #         "arg1",
    #         default_value="arg1_default",
    #         description="description of arg1",
    #     ),
    # )

    ns = "f1"
    ip = "192.168.1.110"
    port = "30000"

    # set Node
    yolo_node = Node(
        package="my_yolo_wrapper",
        namespace=ns,
        executable="boundingbox",
        # output="screen",
        parameters=[
            {
                "img_show": True,
                "timer_period": 0.04,
                "model_path": "/home/dcasl-icryu/ros2_ws/src/AMPF_UbuntuPC/yolov8/runs/detect/train/weights/best.pt",
            }
        ],
        remappings=[
            ("/image", "/" + ns + "/zed_node/rgb/image_rect_color"),
            ("/boundingbox", "/" + ns + "/boundingbox"),
        ],
    )

    leader_distang_node = Node(
        package="pcl_basics",
        namespace=ns,
        executable="leader_distangle",
        # output="screen",
        parameters=[
            {
                "polar_timer_period": 33,  # [ms] 30Hz
                "point_cloud_timer_period": 10,  # [ms] 10Hz
            }
        ],
        remappings=[
            ("/pcl", "/" + ns + "/zed_node/point_cloud/cloud_registered"),
            ("/boundingbox", "/" + ns + "/boundingbox"),
            ("/leader_polar", "/" + ns + "/leader_polar"),
            ("/detected_pc2", "/" + ns + "/detected_pc2"),
            ("/pcl_topic_statistics", "/" + ns + "/pcl_topic_statistics"),
            (
                "/boundingbox_topic_statistics",
                "/" + ns + "/boundingbox_topic_statistics",
            ),
        ],
    )

    desired_formation_node = Node(
        package="formation_controller",
        namespace=ns,
        executable="ampf_df",
        output="screen",
        parameters=[
            {
                "zs12": 1.0,
                "zs13": 1.0,
                "zs23": 1.0,
                "timer_period": 100,  # [ms] 10Hz
            }
        ],
        remappings=[
            ("/desired_formation", "/" + ns + "/desired_formation"),
        ],
    )

    controller_node = Node(
        package="formation_controller",
        namespace=ns,
        executable="ampf_gb_f1",
        output="screen",
        parameters=[
            {
                "timer_period": 100,  # [ms] 10Hz
            }
        ],
        remappings=[
            ("/cmd_vel", "/" + ns + "/cmd_vel"),
            ("/leader_polar", "/" + ns + "/leader_polar"),
            ("/desired_formation", "/" + ns + "/desired_formation"),
            ("/polar_topic_statistics", "/" + ns + "/polar_topic_statistics"),
            ("/df_topic_statistics", "/" + ns + "/df_topic_statistics"),
        ],
    )

    # set Launch file
    zed_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("zed_wrapper"), "/launch", "/zed2i_ip.launch.py"]
        ),
        launch_arguments={
            "cam_ip": ip,
            "cam_port": port,
            "camera_name": ns,
        }.items(),
    )

    # get all actions here
    return LaunchDescription(
        [
            # Set Environment Variable
            # SetEnvironmentVariable(name="", value=""),
            # Declare launch argument that you want to use
            # dc_arg1,
            # Nodes that you want to run
            # run_node_name,
            # Other launch files from other pkgs
            zed_ros_launch,
            yolo_node,
            leader_distang_node,
            controller_node,
            desired_formation_node,
        ]
    )
