import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from datetime import datetime

# Specify files like yaml, xacro, urdf in this format
config_common = os.path.join(
    get_package_share_directory("formation_controller"), "config", "param.yaml"
)


def generate_launch_description():
    desired_formation_node = Node(
        package="formation_controller",
        executable="desired_formation",
        output="screen",
        parameters=[config_common],
    )

    follower1_controller_node = Node(
        package="formation_controller",
        namespace="/f1",
        executable="follower1_controller",
        output="screen",
        parameters=[config_common],
        remappings=[
            # subscriber topic
            ("/polar_coordinate", "/f1/polar_coordinate"),
            ("/lost_leader", "/f1/lost_leader"),
            # publisher
            ("/follower1info", "/f1/follower1info"),
            ("/cmd_vel", "/f1/cmd_vel"),
            ("/nonfiltered_cmd_vel", "/f1/nonfiltered_cmd_vel"),
        ],
    )

    follower2_controller_node = Node(
        package="formation_controller",
        namespace="/f2",
        executable="follower2_controller",
        output="screen",
        parameters=[config_common],
        remappings=[
            # subscriber topic
            ("/polar_coordinate", "/f2/polar_coordinate"),
            ("/lost_leader", "/f2/lost_leader"),
            ("/odom", "/f2/odom"),
            ("/uwb", "/f2/uwb"),
            # publisher topic
            ("/filtered_cmd_vel", "/f2/filtered_cmd_vel"),
            ("/follower2info", "/f2/follower2info"),
            ("/localization", "/f2/localization"),
            ("/filtered_uwb", "/f2/filtered_uwb"),
            ("/cmd_vel", "/f2/cmd_vel"),
            ("/nonfiltered_cmd_vel", "/f2/nonfiltered_cmd_vel"),
        ],
    )
    
    now = datetime.now()
    strnow = now.strftime("%Y_%m_%d-%H_%M_%S")
    data_path = "/home/humble/plotjuggler/data/rosbag2_"
    data_name = data_path + strnow
    
    sim_topic_name = "/f1/odom /f2/odom /f1/polar_coordinate /f2/polar_coordinate /follower_polar /f1/lost_leader /f2/lost_leader /Crazyflie/gps /f1/TurtleBot3Burgerf1/gps /f2/TurtleBot3Burgerf2/gps /f1/TurtleBot3Burgerf1/compass/north_vector /f2/TurtleBot3Burgerf2/compass/north_vector "
    controller_topic_name = "/desired_formation /f1/polar_coordinate /f1/lost_leader /f1/follower1info /f1/cmd_vel /f2/polar_coordinate /f2/lost_leader /f2/uwb /f2/odom /f2/follower2info /f2/cmd_vel /localization /filtered_uwb /filtered_cmd_vel"
    record_topic_name = sim_topic_name + controller_topic_name

    # get all actions here
    return LaunchDescription(
        [
            desired_formation_node, 
            follower1_controller_node,
            follower2_controller_node,
            ExecuteProcess(
                    cmd=["ros2", "bag", "record", "-a", "-o", data_name],
                    shell=True,
                    output="screen",
                ),
        ]
        
    )
