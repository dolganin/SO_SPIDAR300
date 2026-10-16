import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_HERE = os.path.dirname(os.path.abspath(__file__))

def _nodes(context):
    model = LaunchConfiguration("model").perform(context)
    path = model if os.path.isabs(model) else os.path.join(_HERE, model)
    with open(path) as f:
        urdf = f.read()
    urdf = urdf.replace('filename="meshes/', f'filename="file://{os.path.dirname(path)}/meshes/')
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(urdf, value_type=str)}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(_HERE, "view.rviz")]),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="robot_collision.urdf",
                              description="URDF file, relative to this directory or absolute"),
        OpaqueFunction(function=_nodes),
    ])
