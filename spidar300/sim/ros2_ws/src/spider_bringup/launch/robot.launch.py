"""Real robot from the laptop: pi_bridge (UDP to the Raspberry Pi) + robot_state_publisher + optional RViz.

On the Pi: `python3 -m spider_pi.robot`. Drive with `ros2 run spider_gait keyboard_teleop`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESCRIPTION = get_package_share_directory("spider_description")


def generate_launch_description():
    description = ParameterValue(
        Command(["xacro ", os.path.join(DESCRIPTION, "urdf", "spider.urdf.xacro"), " sim:=false"]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("pi_host", default_value="raspberrypi.local",
                              description="Raspberry Pi hostname or IP"),
        DeclareLaunchArgument("pi_port", default_value="5005"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="spider_gait", executable="pi_bridge", output="screen",
             parameters=[{"pi_host": LaunchConfiguration("pi_host"),
                          "pi_port": ParameterValue(LaunchConfiguration("pi_port"), value_type=int)}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": description}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(DESCRIPTION, "rviz", "view.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
