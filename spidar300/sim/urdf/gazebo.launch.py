import os
import re
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_HERE = os.path.dirname(os.path.abspath(__file__))

SERVO_VELOCITY = 5.8

def _controller(joint: str) -> str:
    return (f'  <gazebo>\n'
            f'    <plugin filename="gz-sim-joint-position-controller-system" '
            f'name="gz::sim::systems::JointPositionController">\n'
            f'      <joint_name>{joint}</joint_name>\n'
            f'      <topic>/spider/{joint}/cmd_pos</topic>\n'
            f'      <use_velocity_commands>true</use_velocity_commands>\n'
            f'      <cmd_max>{SERVO_VELOCITY}</cmd_max>\n'
            f'      <cmd_min>{-SERVO_VELOCITY}</cmd_min>\n'
            f'    </plugin>\n'
            f'  </gazebo>\n')

def _nodes(context):
    model = LaunchConfiguration("model").perform(context)
    path = model if os.path.isabs(model) else os.path.join(_HERE, model)
    with open(path) as f:
        urdf = f.read()
    urdf = urdf.replace('filename="meshes/', f'filename="file://{os.path.dirname(path)}/meshes/')
    joints = re.findall(r'<joint name="([^"]+)" type="revolute">', urdf)
    urdf = urdf.replace("</robot>", "".join(_controller(j) for j in joints) + "</robot>")
    bridges = [f"/spider/{j}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double" for j in joints]
    return [
        ExecuteProcess(cmd=["gz", "sim", "-r", "empty.sdf"], output="screen"),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-name", "spider", "-string", urdf, "-z", LaunchConfiguration("z")]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock", *bridges]),
        ExecuteProcess(
            cmd=[sys.executable, os.path.join(_HERE, "walk.py"), "--urdf", path,
                 "--heading", LaunchConfiguration("heading"), "--step", LaunchConfiguration("step"),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen", condition=IfCondition(LaunchConfiguration("walk"))),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="robot.urdf",
                              description="URDF file, relative to this directory or absolute"),
        DeclareLaunchArgument("z", default_value="0.075", description="Spawn height of base_link"),
        DeclareLaunchArgument("walk", default_value="true", description="Start the tripod gait"),
        DeclareLaunchArgument("heading", default_value="-45.0", description="Walking direction, degrees"),
        DeclareLaunchArgument("step", default_value="0.05", description="Stride length, m"),
        OpaqueFunction(function=_nodes),
    ])
