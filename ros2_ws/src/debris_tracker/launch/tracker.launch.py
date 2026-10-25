"""
ros2 launch debris_tracker tracker.launch.py                       # real D405 (realsense2_camera)
ros2 launch debris_tracker tracker.launch.py fake_camera:=true     # closed-loop test, no hardware
ros2 launch debris_tracker tracker.launch.py image_topic:=/twin/camera/image_raw \
    camera_info_topic:=/twin/camera/camera_info                    # Unity twin
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("debris_tracker"), "config", "tracker.yaml")
    args = {
        "image_topic": "/camera/camera/color/image_raw",
        "camera_info_topic": "/camera/camera/color/camera_info",
        "depth_topic": "",
        "world_frame": "",
    }
    decl = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    decl.append(DeclareLaunchArgument("fake_camera", default_value="false"))

    tracker = Node(
        package="debris_tracker", executable="tracker_node", namespace="urex", output="screen",
        # value_type=str so an empty launch argument stays "" instead of being parsed as null
        parameters=[cfg, {k: ParameterValue(LaunchConfiguration(k), value_type=str) for k in args}],
    )
    fake = Node(
        package="debris_tracker", executable="fake_camera_node", output="screen",
        condition=IfCondition(LaunchConfiguration("fake_camera")),
    )
    return LaunchDescription(decl + [tracker, fake])
