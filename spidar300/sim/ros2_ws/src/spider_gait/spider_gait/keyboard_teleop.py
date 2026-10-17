"""Keyboard teleop for the hexapod: walking keys like teleop_twist_keyboard plus stand / lie down.

A key keeps the robot moving while it is held (terminal auto-repeat); after
`hold_timeout` without a key the command drops to zero. Keys work in the Russian
layout too (same physical keys).
"""
from __future__ import annotations

import codecs
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger

HELP = """
  walk            strafe (Shift)
  u   i   o       U   I   O
  j   k   l       J   K   L
  m   ,   .       M   <   >

  i / ,  forward / back      j / l  turn left / right     k / space  stop
  J / L  step left / right   u o m .  arcs                U O M >  diagonals
  t  stand up    b  lie down    q / z  faster / slower    Ctrl-C  quit
"""

# key -> (forward, left, turn), in units of speed / turn
MOVES = {
    "i": (1, 0, 0), ",": (-1, 0, 0), "j": (0, 0, 1), "l": (0, 0, -1),
    "u": (1, 0, 1), "o": (1, 0, -1), "m": (-1, 0, -1), ".": (-1, 0, 1),
    "I": (1, 0, 0), "<": (-1, 0, 0), "J": (0, 1, 0), "L": (0, -1, 0),
    "U": (1, 1, 0), "O": (1, -1, 0), "M": (-1, 1, 0), ">": (-1, -1, 0),
    "k": (0, 0, 0), "K": (0, 0, 0), " ": (0, 0, 0),
}

# Russian layout -> the Latin key on the same physical position
RU = dict(zip("йцукенгшщзхъфывапролджэячсмитьбюЙЦУКЕНГШЩЗХЪФЫВАПРОЛДЖЭЯЧСМИТЬБЮ",
              "qwertyuiop[]asdfghjkl;'zxcvbnm,.QWERTYUIOP{}ASDFGHJKL:\"ZXCVBNM<>"))


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("spider_keyboard")
        self.speed = self.declare_parameter("speed", 0.05).value
        self.turn = self.declare_parameter("turn", 0.4).value
        self.hold_timeout = self.declare_parameter("hold_timeout", 0.6).value
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.shown = None                        # command on the status line; None = redraw
        self.posture_clients = {"t": self.create_client(Trigger, "stand_up"),
                         "b": self.create_client(Trigger, "lie_down")}

    def call(self, key: str) -> None:
        client = self.posture_clients[key]
        if not client.service_is_ready():
            self.say(f"{client.srv_name} not available")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self.say(f.result().message))

    def say(self, text: str) -> None:
        sys.stdout.write(f"\r\x1b[K{text}\r\n")
        sys.stdout.flush()
        self.shown = None

    def status(self, move) -> None:
        sys.stdout.write(f"\r\x1b[Kcmd: forward {move[0] * self.speed:+.3f}  left {move[1] * self.speed:+.3f} m/s  "
                         f"turn {move[2] * self.turn:+.2f} rad/s")
        sys.stdout.flush()

    def publish(self, move) -> None:
        msg = Twist()
        msg.linear.x = move[0] * self.speed
        msg.linear.y = move[1] * self.speed
        msg.angular.z = move[2] * self.turn
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    decoder = codecs.getincrementaldecoder("utf-8")(errors="ignore")
    print(HELP)
    move, last_key = (0, 0, 0), 0.0
    try:
        tty.setraw(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([fd], [], [], 0.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            keys = decoder.decode(os.read(fd, 64)) if ready else ""
            if "\x03" in keys:
                break
            for key in keys:
                key = RU.get(key, key)
                if key in MOVES:
                    move, last_key = MOVES[key], now
                elif key in node.posture_clients:
                    node.call(key)
                elif key in "qz":
                    k = 1.1 if key == "q" else 1 / 1.1
                    node.speed, node.turn = node.speed * k, node.turn * k
                    node.say(f"speed {node.speed:.3f} m/s  turn {node.turn:.2f} rad/s")
                else:
                    node.say(f"unknown key {key!r}")
            if move != (0, 0, 0) and now - last_key > node.hold_timeout:
                move = (0, 0, 0)
            node.publish(move)
            if move != node.shown:
                node.status(move)
                node.shown = move
    finally:
        node.publish((0, 0, 0))
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
