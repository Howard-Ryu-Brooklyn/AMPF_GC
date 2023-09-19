import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Specify files like yaml, xacro, urdf in this format
config_path = os.path.join(
    get_package_share_directory("formation_experiment"), "config", "param.yaml"
)


def generate_launch_description():
    namespace = "/f2"

    image_remap = namespace + "/zed_node/rgb/image_rect_color"
    boundingbox_remap = namespace + "/boundingbox"
    obj_detection_time_remap = namespace + "/obj_detection_time"
    lost_leader_remap = namespace + "/lost_leader"
    pcl_remap = namespace + "/zed_node/point_cloud/cloud_registered"
    cropped_point_cloud_remap = namespace + "/cropped_point_cloud"
    poloar_coordinate_remap = namespace + "/polar_coordinate"
    point_cloud_processing_time_remap = namespace + "/point_cloud_processing_time"

    # set Node
    object_detection_node = Node(
        package="bounding_box",
        executable="bounding_box_node",
        namespace=namespace,
        parameters=[config_path],
        remappings=[
            ("/image", image_remap),
            ("/boundingbox", boundingbox_remap),
            ("/obj_detection_time", obj_detection_time_remap),
            ("/lost_leader", lost_leader_remap),
        ],
    )

    # set Node
    point_cloud_processing_node = Node(
        package="point_cloud_processing",
        executable="point_cloud_processing",
        namespace=namespace,
        parameters=[config_path],
        remappings=[
            ("/pcl", pcl_remap),
            ("/boundingbox", boundingbox_remap),
            ("/cropped_point_cloud", cropped_point_cloud_remap),
            ("/poloar_coordinate", poloar_coordinate_remap),
            ("/point_cloud_processing_time", point_cloud_processing_time_remap),
        ],
    )

    ip = LaunchConfiguration("ip", default="192.168.1.120")
    port = LaunchConfiguration("port", default="30000")
    camera_model = LaunchConfiguration("camera_model", default="zed2i")

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("zed_wrapper"),
                "/launch/include/zed_camera_ip.launch.py",
            ]
        ),
        launch_arguments={
            "camera_name": "f2",
            "camera_model": camera_model,
            "cam_ip": ip,
            "cam_port": port,
        }.items(),
    )

    # get all actions here
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="camera_model",
                default_value="zed2i",
                description="model of zed camera",
            ),
            DeclareLaunchArgument(
                name="ip", default_value="192.168.1.120", description="ip of zed camera"
            ),
            DeclareLaunchArgument(
                name="port",
                default_value="30000",
                description="port of zed camera",
            ),
            zed_wrapper_launch,
            object_detection_node,
            point_cloud_processing_node,
        ]
    )
