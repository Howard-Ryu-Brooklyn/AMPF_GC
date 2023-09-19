import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
    package_dir = get_package_share_directory('formation_simulation')
    cf_robot_description_path = os.path.join(package_dir, 'resource', 'crazyflie_webots.urdf')
    
    crazyflie_controller = WebotsController(
        robot_name='Crazyflie',
        parameters=[
            {'robot_description': cf_robot_description_path,
             'use_sim_time': True
             },
        ],
        respawn=True
    )
    
    return LaunchDescription([
        crazyflie_controller,
    ])