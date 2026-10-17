"""Gazebo world + spider + ros_gz bridge + /joint_command adapter + gait.

/cmd_vel -> spider_gait -> /joint_command -> joint_command_adapter -> /spider/<joint>/cmd_pos
-> ros_gz_bridge -> gz JointPositionController; /joint_states and /clock come back the same way.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESCRIPTION = get_package_share_directory("spider_description")
BRINGUP = get_package_share_directory("spider_bringup")
LEGS_YAML = os.path.join(DESCRIPTION, "config", "legs.yaml")


def joint_names(path=LEGS_YAML):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return [f"leg_{leg['name']}_{j}" for leg in cfg["legs"] for j in cfg["joints"]]


def bridge_config():
    """ros_gz_bridge entries generated from the joint list in legs.yaml."""
    entries = [
        {"ros_topic_name": "/clock", "gz_topic_name": "/clock", "ros_type_name": "rosgraph_msgs/msg/Clock",
         "gz_type_name": "gz.msgs.Clock", "direction": "GZ_TO_ROS"},
        {"ros_topic_name": "/joint_states", "gz_topic_name": "/joint_states",
         "ros_type_name": "sensor_msgs/msg/JointState", "gz_type_name": "gz.msgs.Model", "direction": "GZ_TO_ROS"},
    ]
    entries += [{"ros_topic_name": f"/spider/{j}/cmd_pos", "gz_topic_name": f"/spider/{j}/cmd_pos",
                 "ros_type_name": "std_msgs/msg/Float64", "gz_type_name": "gz.msgs.Double",
                 "direction": "ROS_TO_GZ"} for j in joint_names()]
    return entries


def _bridge(context):
    path = os.path.join(tempfile.gettempdir(), "spider_bridge.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(bridge_config(), f, sort_keys=False)
    return [Node(package="ros_gz_bridge", executable="parameter_bridge", name="spider_bridge", output="screen",
                 parameters=[{"config_file": path, "use_sim_time": True}])]


def generate_launch_description():
    sim_time = {"use_sim_time": True}
    description = ParameterValue(
        Command(["xacro ", os.path.join(DESCRIPTION, "urdf", "spider.urdf.xacro")]), value_type=str)
    gz_args = PythonExpression(["'-r ' + '", LaunchConfiguration("world"), "'",
                                " + ('' if '", LaunchConfiguration("gui"), "' == 'true' else ' -s')"])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=os.path.join(DESCRIPTION, "worlds", "flat.sdf")),
        DeclareLaunchArgument("z", default_value="0.25",
                              description="Spawn height of base_link, m; legs fold into rest_pose while falling"),
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI (false = server only)"),
        DeclareLaunchArgument("gait", default_value="true", description="Start the gait node"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("gait_params", default_value=os.path.join(BRINGUP, "config", "gait.yaml")),

        # package://spider_description/... in the URDF resolves through this path
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(DESCRIPTION)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()),

        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": description}, sim_time]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "spider", "-z", LaunchConfiguration("z")]),
        OpaqueFunction(function=_bridge),

        Node(package="spider_gait", executable="joint_command_adapter", output="screen", parameters=[sim_time]),
        Node(package="spider_gait", executable="gait_node", name="spider_gait", output="screen",
             parameters=[LaunchConfiguration("gait_params"), sim_time],
             condition=IfCondition(LaunchConfiguration("gait"))),
        Node(package="rviz2", executable="rviz2", output="screen", parameters=[sim_time],
             arguments=["-d", os.path.join(DESCRIPTION, "rviz", "view.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
