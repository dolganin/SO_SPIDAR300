import argparse
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))

LEGS = (1, 2, 3, 4, 5, 6)
PARTS = ("coxa", "femur", "tibia")
FOOT_IN_TIBIA = np.array([0.1111, 0.1255, -0.0193, 1.0])
STAND_TIME = 2.0
STANCE_REACH = 0.15
STANCE_HEIGHT = 0.10

def joint_name(leg: int, part: str) -> str:
    return f"Leg_{leg}_{part}_joint"

JOINT_NAMES = tuple(joint_name(leg, part) for leg in LEGS for part in PARTS)

def _origin(joint: ET.Element) -> np.ndarray:
    o = joint.find("origin")
    x, y, z = map(float, o.get("xyz").split())
    r, p, yw = map(float, o.get("rpy").split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yw), math.sin(yw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0.0, 0.0, 0.0, 1.0]])

def _rot_z(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

def load_chains(urdf: str) -> dict[int, list[np.ndarray]]:
    joints = {j.get("name"): j for j in ET.parse(urdf).getroot().findall("joint")}
    return {leg: [_origin(joints[joint_name(leg, part)]) for part in PARTS] for leg in LEGS}

def foot_position(chain: list[np.ndarray], q: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    for origin, angle in zip(chain, q):
        t = t @ origin @ _rot_z(angle)
    return (t @ FOOT_IN_TIBIA)[:3]

def solve_leg(chain: list[np.ndarray], target: np.ndarray, q: np.ndarray) -> np.ndarray:
    q = q.copy()
    for _ in range(10):
        p = foot_position(chain, q)
        err = target - p
        if np.linalg.norm(err) < 1e-5:
            break
        jac = np.column_stack([
            (foot_position(chain, q + 1e-5 * np.eye(3)[i]) - p) / 1e-5 for i in range(3)])
        q += jac.T @ np.linalg.solve(jac @ jac.T + 1e-6 * np.eye(3), err)
    return q

def stance_target(chain: list[np.ndarray], reach: float = STANCE_REACH, height: float = STANCE_HEIGHT) -> np.ndarray:
    hip = chain[0][:3, 3]
    out = foot_position(chain, np.zeros(3)) - hip
    radial = out[:2] / np.linalg.norm(out[:2])
    return np.array([hip[0] + reach * radial[0], hip[1] + reach * radial[1], -height])

def stance_joints(chains: dict[int, list[np.ndarray]]) -> dict[str, float]:
    out = {}
    for leg in LEGS:
        q = np.zeros(3)
        for _ in range(5):
            q = solve_leg(chains[leg], stance_target(chains[leg]), q)
        for part, angle in zip(PARTS, q):
            out[joint_name(leg, part)] = float(angle)
    return out

class TripodGait:
    def __init__(self, urdf: str, heading_deg: float, step: float, lift: float, period: float):
        self.chains = load_chains(urdf)
        self.step, self.lift, self.period = step, lift, period
        a = math.radians(heading_deg)
        self.forward = np.array([math.cos(a), math.sin(a), 0.0])
        stance = stance_joints(self.chains)
        self.q = {leg: np.array([stance[joint_name(leg, part)] for part in PARTS]) for leg in LEGS}
        self.neutral = {leg: foot_position(self.chains[leg], self.q[leg]) for leg in LEGS}
        by_angle = sorted(LEGS, key=lambda leg: math.atan2(self.chains[leg][0][1, 3], self.chains[leg][0][0, 3]))
        self.phase = {leg: 0.5 * (i % 2) for i, leg in enumerate(by_angle)}

    def _offset(self, phase: float, scale: float) -> np.ndarray:
        if phase < 0.5:
            along, up = 0.5 - phase / 0.5, 0.0
        else:
            s = (phase - 0.5) / 0.5
            along, up = -0.5 * math.cos(math.pi * s), math.sin(math.pi * s)
        return scale * (self.step * along * self.forward + np.array([0.0, 0.0, self.lift * up]))

    def joints(self, t: float) -> dict[str, float]:
        walk_t = max(0.0, t - STAND_TIME)
        scale = min(1.0, walk_t / self.period)
        out = {}
        for leg in LEGS:
            phase = (walk_t / self.period + self.phase[leg]) % 1.0
            target = self.neutral[leg] + self._offset(phase, scale)
            self.q[leg] = solve_leg(self.chains[leg], target, self.q[leg])
            for part, angle in zip(PARTS, self.q[leg]):
                out[joint_name(leg, part)] = float(angle)
        return out

def add_gait_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--urdf", default=os.path.join(_HERE, "robot.urdf"))
    parser.add_argument("--heading", type=float, default=-45.0, help="Walking direction in base_link, degrees")
    parser.add_argument("--step", type=float, default=0.05, help="Stride length, m")
    parser.add_argument("--lift", type=float, default=0.025, help="Foot lift height, m")
    parser.add_argument("--period", type=float, default=1.0, help="Gait cycle, s")

def main() -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64

    parser = argparse.ArgumentParser(description="Open-loop tripod gait for the hexapod in Gazebo")
    add_gait_args(parser)
    parser.add_argument("--rate", type=float, default=50.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node("spider_walk")
    gait = TripodGait(args.urdf, args.heading, args.step, args.lift, args.period)
    pubs = {name: node.create_publisher(Float64, f"/spider/{name}/cmd_pos", 10) for name in JOINT_NAMES}
    start = None

    def tick() -> None:
        nonlocal start
        now = node.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:
            return
        if start is None:
            start = now
        for name, angle in gait.joints(now - start).items():
            pubs[name].publish(Float64(data=angle))

    node.create_timer(1.0 / args.rate, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
