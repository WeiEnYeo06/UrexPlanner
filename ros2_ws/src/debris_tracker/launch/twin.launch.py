"""
Unity digital twin <-> ROS 2, through rosbridge (Unity side: TwinRosBridge.cs on the Twin object).

ros2 launch debris_tracker twin.launch.py                   # rosbridge (port 9090) + tracker, then press Play in Unity
ros2 launch debris_tracker twin.launch.py tracker:=false    # rosbridge only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("debris_tracker"), "config", "tracker.yaml")
    rosbridge_launch = os.path.join(get_package_share_directory("rosbridge_server"), "launch",
                                    "rosbridge_websocket_launch.xml")
    decl = [
        DeclareLaunchArgument("port", default_value="9090"),
        DeclareLaunchArgument("tracker", default_value="true"),
    ]

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch),
        launch_arguments={"port": LaunchConfiguration("port")}.items(),
    )
    tracker = Node(
        package="debris_tracker", executable="tracker_node", namespace="urex", output="screen",
        parameters=[cfg, {
            "image_topic": "/twin/camera/image_raw",
            "camera_info_topic": "/twin/camera/camera_info",
            "world_frame": "world",            # Unity publishes TF world -> d405_color_optical_frame
            # twin debris colour: TwinPlayback.debrisColor (0.20, 0.80, 0.30) = OpenCV HSV ~(65, 191, 204)
            "hsv_lower": [50, 100, 50],
            "hsv_upper": [80, 255, 255],
        }],
        condition=IfCondition(LaunchConfiguration("tracker")),
    )
    return LaunchDescription(decl + [rosbridge, tracker])
