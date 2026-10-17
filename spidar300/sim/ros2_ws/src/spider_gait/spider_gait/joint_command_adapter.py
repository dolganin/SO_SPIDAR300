"""/joint_command (JointState) -> one Float64 position command per joint for gz-sim.

Velocities in the command are ignored: the Gazebo position controllers track angles.
A servo driver replaces this node on the real robot.
"""
from __future__ import annotations

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .kinematics import Hexapod


class JointCommandAdapter(Node):
    def __init__(self):
        super().__init__("joint_command_adapter")
        default_cfg = os.path.join(get_package_share_directory("spider_description"), "config", "legs.yaml")
        self.declare_parameter("config", default_cfg)
        self.declare_parameter("topic_format", "/spider/{joint}/cmd_pos")
        fmt = self.get_parameter("topic_format").value
        names = Hexapod.load(self.get_parameter("config").value).joint_names
        self.pubs = {n: self.create_publisher(Float64, fmt.format(joint=n), 10) for n in names}
        self.unknown: set[str] = set()
        self.create_subscription(JointState, "joint_command", self._on_command, 10)

    def _on_command(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            pub = self.pubs.get(name)
            if pub is not None:
                pub.publish(Float64(data=float(position)))
            elif name not in self.unknown:
                self.unknown.add(name)
                self.get_logger().warn(f"ignoring unknown joint '{name}'")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
