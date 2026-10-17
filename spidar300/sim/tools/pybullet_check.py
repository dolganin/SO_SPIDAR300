"""Checks spider_description and spider_gait in pybullet, without ROS or Gazebo.

  kinematics  URDF foot frames vs spider_gait.kinematics for random poses
  drop        spawned like in Gazebo (zero pose, 25 cm up) with servos holding the
              rest pose: must land on its belly and come to rest
  walk        from lying: stand up, a sequence of /cmd_vel-like commands, lie down,
              stand up again; reports travel, body height, tilt and stance-foot slip
"""
from __future__ import annotations

import argparse
import math
import sys
import tempfile
from pathlib import Path

import numpy as np
import pybullet as p
import xacro

_WS = Path(__file__).resolve().parents[1] / "ros2_ws" / "src"
_DESC = _WS / "spider_description"
sys.path.insert(0, str(_WS / "spider_gait"))

from spider_gait.gait import TRIPODS, GaitParams, JointRateLimiter, TripodGait  # noqa: E402
from spider_gait.kinematics import Hexapod  # noqa: E402

DT = 1.0 / 240.0
GAIT_RATE = 50.0


def build_urdf() -> str:
    doc = xacro.process_file(str(_DESC / "urdf" / "spider.urdf.xacro"),
                             mappings={"sim": "false", "config_dir": str(_DESC / "config")})
    text = doc.toprettyxml(indent="  ").replace("package://spider_description", str(_DESC))
    f = tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False)
    f.write(text)
    f.close()
    return f.name


class Sim:
    def __init__(self, urdf: str, robot: Hexapod, gui: bool, z: float, fixed: bool = False):
        p.connect(p.GUI if gui else p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(DT)
        plane = p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE))
        p.changeDynamics(plane, -1, lateralFriction=1.0)
        self.id = p.loadURDF(urdf, [0, 0, z], flags=p.URDF_USE_INERTIA_FROM_FILE, useFixedBase=fixed)
        self.robot = robot
        self.index = {}
        for i in range(p.getNumJoints(self.id)):
            info = p.getJointInfo(self.id, i)
            self.index[info[1].decode()] = i
            self.index[info[12].decode()] = i
            p.changeDynamics(self.id, i, lateralFriction=1.0)
            if info[2] == p.JOINT_REVOLUTE:
                p.setJointMotorControl2(self.id, i, p.VELOCITY_CONTROL, force=0)
        self.joints = [self.index[n] for n in robot.joint_names]

    def command(self, q: np.ndarray, effort: float, kp: float = 0.4, kd: float = 1.0) -> None:
        p.setJointMotorControlArray(self.id, self.joints, p.POSITION_CONTROL, targetPositions=list(q),
                                    forces=[effort] * len(q), positionGains=[kp] * len(q),
                                    velocityGains=[kd] * len(q))

    def run(self, seconds: float) -> None:
        for _ in range(int(round(seconds / DT))):
            p.stepSimulation()

    def base(self):
        """base_link origin (not its centre of mass) and roll/pitch/yaw."""
        com, orn = p.getBasePositionAndOrientation(self.id)
        local = p.getDynamicsInfo(self.id, -1)[3]
        r = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        return np.array(com) - r @ np.array(local), np.array(p.getEulerFromQuaternion(orn))

    def foot(self, leg: str) -> np.ndarray:
        return np.array(p.getLinkState(self.id, self.index[f"leg_{leg}_foot"])[4])

    def close(self) -> None:
        p.disconnect()


def check_kinematics(urdf: str, robot: Hexapod) -> None:
    sim = Sim(urdf, robot, False, 0.0, fixed=True)
    rng = np.random.default_rng(0)
    err = 0.0
    for _ in range(200):
        q = {leg: rng.uniform([robot.limits[j][0] for j in robot.joints],
                              [robot.limits[j][1] for j in robot.joints]) for leg in robot.legs}
        for leg in robot.legs:
            for j, a in zip(robot.joints, q[leg]):
                p.resetJointState(sim.id, sim.index[f"leg_{leg}_{j}"], a)
        for leg in robot.legs:
            err = max(err, np.linalg.norm(sim.foot(leg) - robot.foot(leg, q[leg])))
    sim.close()
    print(f"kinematics: max |URDF foot - FK foot| = {err * 1e3:.4f} mm")


SPAWN_Z = 0.25


def rest_command(robot: Hexapod) -> np.ndarray:
    return np.tile(robot.rest_pose, len(robot.legs))


def check_drop(urdf: str, robot: Hexapod, effort: float, gui: bool) -> None:
    sim = Sim(urdf, robot, gui, SPAWN_Z)
    sim.command(rest_command(robot), effort)
    sim.run(3.0)
    pos0, rpy0 = sim.base()
    sim.run(2.0)
    pos1, rpy1 = sim.base()
    v = p.getBaseVelocity(sim.id)
    sim.close()
    print(f"drop: base z {pos1[2]:.3f} m, drift over 2 s {np.linalg.norm(pos1 - pos0) * 1e3:.2f} mm, "
          f"tilt change {math.degrees(np.abs(rpy1 - rpy0).max()):.2f} deg, "
          f"|v| {np.linalg.norm(v[0]) * 1e3:.2f} mm/s, |w| {np.linalg.norm(v[1]):.4f} rad/s")


def check_walk(urdf: str, robot: Hexapod, params: GaitParams, effort: float, gui: bool) -> None:
    sim = Sim(urdf, robot, gui, SPAWN_Z)
    sim.command(rest_command(robot), effort)
    sim.run(2.0)
    gait = TripodGait(robot, params)
    limiter = JointRateLimiter(robot.max_velocity)
    sub = int(round(1.0 / (GAIT_RATE * DT)))
    dt = sub * DT

    def drive(cmd, seconds, label):
        gait.set_command(*cmd)
        pos0, rpy0 = sim.base()
        tilt, slip = 0.0, 0.0
        anchor: dict[str, np.ndarray] = {}
        for _ in range(int(round(seconds * GAIT_RATE))):
            q = gait.step(dt)
            qv, _ = limiter(np.concatenate([q[l] for l in robot.legs]), dt)
            sim.command(qv, effort)
            sim.run(dt)
            _, rpy = sim.base()
            tilt = max(tilt, math.degrees(max(abs(rpy[0]), abs(rpy[1]))))
            swing = TRIPODS[gait._swing_group()] if gait.walking else ()
            for leg in robot.legs:
                f = sim.foot(leg)
                if leg in swing or f[2] > robot.foot_radius + 0.004:
                    anchor.pop(leg, None)
                elif leg not in anchor:
                    anchor[leg] = f
                else:
                    slip = max(slip, np.linalg.norm(f[:2] - anchor[leg][:2]))
        pos1, rpy1 = sim.base()
        d = pos1 - pos0
        yaw0 = rpy0[2]
        fwd = d[0] * math.cos(yaw0) + d[1] * math.sin(yaw0)
        side = -d[0] * math.sin(yaw0) + d[1] * math.cos(yaw0)
        dyaw = math.degrees(math.remainder(rpy1[2] - yaw0, math.tau))
        print(f"{label:14s} fwd {fwd * 100:+6.1f} cm  side {side * 100:+6.1f} cm  yaw {dyaw:+6.1f} deg  "
              f"height {pos1[2] * 100:5.1f} cm  max tilt {tilt:4.1f} deg  max stance slip {slip * 1e3:4.1f} mm")

    drive((0, 0, 0), 1.0, "lying")
    gait.stand_up()
    drive((0.05, 0, 0), params.tuck_time + params.rise_time + 3.0, "stand up")
    print(f"  mode {gait.mode.value}")
    drive((0, 0, 0), 3.0, "hold")
    drive((0.05, 0, 0), 6.0, "forward 5cm/s")
    drive((0, 0, 0), 2.0, "stop")
    drive((-0.05, 0, 0), 4.0, "back 5cm/s")
    drive((0, 0.05, 0), 4.0, "left 5cm/s")
    drive((0, 0, 0.4), 4.0, "turn 0.4rad/s")
    drive((0.05, 0, 0.3), 4.0, "arc")
    drive((0, 0, 0), 2.0, "stop")
    print(f"walking after stop: {gait.walking}")
    gait.lie_down()
    drive((0, 0, 0), 2 * params.period + params.tuck_time + params.rise_time + 1.0, "lie down")
    print(f"  mode {gait.mode.value}")
    drive((0, 0, 0), 2.0, "lying")
    gait.stand_up()
    drive((0, 0, 0), params.tuck_time + params.rise_time + 3.0, "stand up again")
    print(f"  mode {gait.mode.value}")
    drive((0.05, 0, 0), 4.0, "forward 5cm/s")
    sim.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("checks", nargs="*", default=["kinematics", "drop", "walk"])
    parser.add_argument("--gui", action="store_true")
    for name, default in vars(GaitParams()).items():
        parser.add_argument(f"--{name.replace('_', '-')}", type=float, default=default)
    args = parser.parse_args()

    robot = Hexapod.load(_DESC / "config" / "legs.yaml")
    import yaml
    effort = yaml.safe_load(open(_DESC / "config" / "legs.yaml"))["servo"]["effort"]
    params = GaitParams(**{k: getattr(args, k) for k in vars(GaitParams())})
    urdf = build_urdf()
    if "kinematics" in args.checks:
        check_kinematics(urdf, robot)
    if "drop" in args.checks:
        check_drop(urdf, robot, effort, args.gui)
    if "walk" in args.checks:
        check_walk(urdf, robot, params, effort, args.gui)


if __name__ == "__main__":
    main()
