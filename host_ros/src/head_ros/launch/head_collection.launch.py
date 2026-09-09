"""Launch the bridge, safe session collector, and optional operator GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    data_root = LaunchConfiguration("data_root")
    show_gui = LaunchConfiguration("gui")
    return LaunchDescription([
        DeclareLaunchArgument("data_root", default_value=""),
        DeclareLaunchArgument("gui", default_value="true"),
        Node(package="head_ros", executable="head_bridge",
             name="head_bridge", output="screen"),
        Node(package="head_ros", executable="head_proprioception_collector",
             name="proprioception_collector", output="screen",
             parameters=[{"data_root": data_root}]),
        Node(package="head_ros", executable="head_proprioception_gui",
             name="proprioception_collection_gui", output="screen",
             condition=IfCondition(show_gui)),
    ])
