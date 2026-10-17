"""/cmd_vel -> tripod gait -> /joint_command (angles and velocities of all 18 joints).

Starts lying down; services ~/stand_up and ~/lie_down (std_srvs/Trigger) switch posture.
/cmd_vel is ignored unless standing.
"""
from __future__ import annotations

import os
from dataclasses import fields

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .gait import GaitParams, JointRateLimiter, TripodGait
from .kinematics import Hexapod

MAX_DT = 0.1


class GaitNode(Node):
    def __init__(self):
        super().__init__("spider_gait")
        default_cfg = os.path.join(get_package_share_directory("spider_description"), "config", "legs.yaml")
        self.declare_parameter("config", default_cfg)
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("cmd_timeout", 0.5)
        for f in fields(GaitParams):
            self.declare_parameter(f.name, f.default)

        self.robot = Hexapod.load(self.get_parameter("config").value)
        self.gait = TripodGait(self.robot, self._read_params())
        self.limiter = JointRateLimiter(self.robot.max_velocity)
        self.names = self.robot.joint_names
        self.add_on_set_parameters_callback(self._on_params)

        self.last_cmd_time: Time | None = None
        self.last_tick: Time | None = None

        self.pub = self.create_publisher(JointState, "joint_command", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_service(Trigger, "stand_up", lambda req, res: self._posture(res, self.gait.stand_up))
        self.create_service(Trigger, "lie_down", lambda req, res: self._posture(res, self.gait.lie_down))
        self.create_timer(1.0 / self.get_parameter("rate").value, self._tick)
        self.mode = None

    def _read_params(self) -> GaitParams:
        return GaitParams(**{f.name: float(self.get_parameter(f.name).value) for f in fields(GaitParams)})

    def _on_params(self, params) -> SetParametersResult:
        names = {f.name for f in fields(GaitParams)}
        updates = {p.name: float(p.value) for p in params if p.name in names}
        if updates:
            for k, v in updates.items():
                setattr(self.gait.params, k, v)
            self.get_logger().info(f"gait parameters: {updates}")
        return SetParametersResult(successful=True)

    def _posture(self, res: Trigger.Response, action) -> Trigger.Response:
        res.success, res.message = action()
        self.get_logger().info(res.message)
        return res

    def _on_cmd(self, msg: Twist) -> None:
        self.gait.set_command(msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return                                   # /clock not running yet
        if self.limiter.last is None:
            # The robot starts in rest_pose (Gazebo initial_position, servo driver on hardware).
            self.limiter.last = np.concatenate([self.gait.q[leg] for leg in self.robot.legs])

        dt = 0.0 if self.last_tick is None else min(MAX_DT, (now - self.last_tick).nanoseconds * 1e-9)
        self.last_tick = now
        if dt <= 0.0:
            return

        timeout = self.get_parameter("cmd_timeout").value
        if self.last_cmd_time is not None and (now - self.last_cmd_time).nanoseconds * 1e-9 > timeout:
            self.gait.set_command(0.0, 0.0, 0.0)
            self.last_cmd_time = None

        q = self.gait.step(dt)
        if self.gait.mode is not self.mode:
            self.mode = self.gait.mode
            self.get_logger().info(f"mode: {self.mode.value}")
        positions, velocities = self.limiter(np.concatenate([q[leg] for leg in self.robot.legs]), dt)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = self.names
        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
