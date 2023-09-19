# Copyright 2022 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Camera model (force value)
    camera_name = LaunchConfiguration("ns", default="f1")
    camera_model = LaunchConfiguration("camera_model", default="zed2i")
    ip = LaunchConfiguration("cam_ip", default="192.168.1.120")
    port = LaunchConfiguration("cam_port", default="30000")

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("zed_wrapper"),
                "/launch/include/zed_camera_ip.launch.py",
            ]
        ),
        launch_arguments={
            "camera_name": camera_name,
            "camera_model": camera_model,
            "cam_ip": ip,
            "cam_port": port,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="cam_ip",
                default_value="192.168.1.120",
                description="An array containing the ip address of the cameras, e.g. [192.168.1.120,  ]",
            ),
            DeclareLaunchArgument(
                "cam_port",
                default_value="30000",
                description="An array containing the ip address of the cameras, e.g. [30000,  ]",
            ),
            DeclareLaunchArgument(
                "camera_model",
                default_value="zed2i",
                description="An array containing the ip address of the cameras, e.g. [f2,  ]",
            ),
            DeclareLaunchArgument(
                "ns",
                default_value="f1",
                description="An array containing the ip address of the cameras, e.g. [f2,  ]",
            ),
            zed_wrapper_launch,
        ]
    )
