"""Laptop side of the real robot: ROS <-> UDP to the Raspberry Pi running spider_pi.robot.

  /cmd_vel           -> {"type": "cmd"} resent at cmd_rate (the Pi stops after its cmd_timeout)
  ~stand_up/lie_down -> {"type": "stand_up" / "lie_down"} (same service names as gait_node)
  gait parameters    -> {"type": "set"} on `ros2 param set /pi_bridge <name> <value>`
  {"type": "state"}  -> /joint_states (measured) and /joint_command (what the Pi sends to the servos)

Message format: pi/spider_pi/protocol.py in the repository.
"""
from __future__ import annotations

import json
import socket
import threading
from dataclasses import fields

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .gait import GaitParams

REPLY_TIMEOUT = 1.0
CMD_HOLD = 0.5               # forward /cmd_vel for this long after the last message


class PiBridge(Node):
    def __init__(self):
        super().__init__("pi_bridge")
        self.host = self.declare_parameter("pi_host", "raspberrypi.local").value
        self.port = int(self.declare_parameter("pi_port", 5005).value)
        rate = float(self.declare_parameter("cmd_rate", 10.0).value)
        self.gait_names = {f.name for f in fields(GaitParams)}
        for f in fields(GaitParams):
            self.declare_parameter(f.name, f.default)
        self.add_on_set_parameters_callback(self._on_params)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.addr = None
        self.next_id = 0
        self.replies: dict[int, dict] = {}
        self.reply_cv = threading.Condition()
        self.mode = None
        self.alive = True

        self.cmd = Twist()
        self.cmd_time = None
        self.state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.command_pub = self.create_publisher(JointState, "joint_command", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_service(Trigger, "stand_up", lambda req, res: self._request(res, {"type": "stand_up"}))
        self.create_service(Trigger, "lie_down", lambda req, res: self._request(res, {"type": "lie_down"}))
        self.create_timer(1.0 / rate, self._send_cmd)
        self.receiver = threading.Thread(target=self._receive, daemon=True)
        self.receiver.start()
        self.get_logger().info(f"talking to {self.host}:{self.port}")

    # -- sending ---------------------------------------------------------------------

    def _send(self, msg: dict) -> bool:
        try:
            if self.addr is None:
                self.addr = (socket.gethostbyname(self.host), self.port)
            self.sock.sendto(json.dumps(msg).encode(), self.addr)
            return True
        except OSError as e:
            self.addr = None
            self.get_logger().warn(f"cannot reach {self.host}:{self.port}: {e}", throttle_duration_sec=5.0)
            return False

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = msg
        self.cmd_time = self.get_clock().now()

    def _send_cmd(self) -> None:
        cmd = self.cmd
        if self.cmd_time is None or (self.get_clock().now() - self.cmd_time).nanoseconds * 1e-9 > CMD_HOLD:
            cmd = Twist()
        self._send({"type": "cmd", "vx": cmd.linear.x, "vy": cmd.linear.y, "wz": cmd.angular.z})

    def _call(self, msg: dict) -> dict | None:
        with self.reply_cv:
            self.next_id += 1
            msg = dict(msg, id=self.next_id)
        if not self._send(msg):
            return None
        with self.reply_cv:
            self.reply_cv.wait_for(lambda: msg["id"] in self.replies, REPLY_TIMEOUT)
            return self.replies.pop(msg["id"], None)

    def _request(self, res: Trigger.Response, msg: dict) -> Trigger.Response:
        reply = self._call(msg)
        if reply is None:
            res.success, res.message = False, f"no reply from {self.host}:{self.port}"
        else:
            res.success, res.message = bool(reply.get("ok")), str(reply.get("message", ""))
        self.get_logger().info(res.message)
        return res

    def _on_params(self, params) -> SetParametersResult:
        updates = {p.name: float(p.value) for p in params if p.name in self.gait_names}
        if updates:
            # don't block the parameter service on the network; the Pi logs what it applied
            self._send({"type": "set", "id": 0, "params": updates})
        return SetParametersResult(successful=True)

    # -- receiving -------------------------------------------------------------------

    def _receive(self) -> None:
        while self.alive:
            try:
                data, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                if not self.alive:
                    return
                continue
            try:
                msg = json.loads(data.decode())
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            kind = msg.get("type")
            if kind == "reply":
                with self.reply_cv:
                    self.replies[msg.get("id")] = msg
                    self.reply_cv.notify_all()
            elif kind == "state":
                self._on_state(msg)

    def _on_state(self, msg: dict) -> None:
        if msg.get("mode") != self.mode:
            self.mode = msg.get("mode")
            self.get_logger().info(f"robot mode: {self.mode}")
        stamp = self.get_clock().now().to_msg()
        for pub, key in ((self.state_pub, "position"), (self.command_pub, "command")):
            js = JointState()
            js.header.stamp = stamp
            js.name = [str(n) for n in msg.get("names", [])]
            js.position = [float(x) for x in msg.get(key, [])]
            pub.publish(js)

    def destroy_node(self):
        self.alive = False
        self.sock.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
