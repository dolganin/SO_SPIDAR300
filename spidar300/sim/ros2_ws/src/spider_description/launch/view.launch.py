"""RViz + joint_state_publisher_gui: check every joint's direction, zero and limits."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    share = get_package_share_directory("spider_description")
    description = ParameterValue(
        Command(["xacro ", os.path.join(share, "urdf", "spider.urdf.xacro"), " sim:=false"]), value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "view.rviz")]),
    ])
