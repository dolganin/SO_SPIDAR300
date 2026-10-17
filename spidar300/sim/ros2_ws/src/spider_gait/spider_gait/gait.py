"""Velocity-driven tripod gait with lie-down / stand-up, independent of ROS.

Feet are planned in base_link. A stance foot moves by -(v + w x r) so the body
travels with (v, w); a swing foot travels to its next touchdown point
neutral + (v + w x neutral) * T_stance / 2 with a sine-shaped lift. Joint angles
come from the analytic IK; joint speed is limited by JointRateLimiter.

The robot starts LYING on its belly with the legs spread out (rest_pose in
legs.yaml). stand_up(): joints blend to feet just above the ground at rise_reach,
then the feet press straight down to body_height, then the gait steps the feet in
to stance_reach. lie_down() runs the same in reverse.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from .kinematics import Hexapod

TRIPODS = (("lf", "rm", "lr"), ("rf", "lm", "rr"))


class Mode(str, Enum):
    LYING = "lying"
    RISING = "rising"
    STANDING = "standing"
    WIDENING = "widening"       # stepping out to rise_reach before lowering
    LOWERING = "lowering"


@dataclass
class GaitParams:
    body_height: float = 0.09       # base_link origin above the ground, m
    stance_reach: float = 0.15      # horizontal coxa axis -> foot distance when standing, m
    step_length: float = 0.06       # longest foot stroke per stance, m
    period: float = 0.8             # full cycle: one swing per tripod, s
    lift_height: float = 0.03       # foot lift at mid-swing, m
    max_linear: float = 0.10        # m/s
    max_angular: float = 0.6        # rad/s
    rise_reach: float = 0.19        # foot reach while pressing the body up / down, m
    lying_height: float = 0.0       # base_link origin above the ground when lying, m
    foot_clearance: float = 0.005   # feet above the ground when lying, m
    tuck_time: float = 1.5          # rest pose <-> feet at rise_reach, s
    rise_time: float = 2.0          # body ground <-> body_height, s
    max_height_rate: float = 0.03   # how fast a changed body_height is applied, m/s
    settle_tolerance: float = 0.002 # feet this close to neutral count as settled, m


def _smooth(s: float) -> float:
    return 0.5 - 0.5 * math.cos(math.pi * min(1.0, max(0.0, s)))


class _Segment:
    """Timed move: either a joint-space blend or a vertical foot press in base_link."""

    def __init__(self, duration: float, joints=None, feet_z: tuple[float, float] | None = None):
        self.duration, self.t = max(duration, 1e-6), 0.0
        self.joints, self.feet_z = joints, feet_z
        self.start: dict[str, np.ndarray] | None = None

    def done(self) -> bool:
        return self.t >= self.duration


class TripodGait:
    def __init__(self, robot: Hexapod, params: GaitParams | None = None):
        self.robot = robot
        self.params = params or GaitParams()
        self.cmd = np.zeros(3)                       # requested vx, vy, wz in base_link, before limits
        self.mode = Mode.LYING
        self.walking = False
        self.phase = 0.0
        self.segments: list[_Segment] = []
        self.rest = {leg: robot.rest_pose.copy() for leg in robot.legs}
        self.q = {leg: q.copy() for leg, q in self.rest.items()}
        self.height = robot.foot_radius - self.lying_foot_z()
        self.reach = self.params.stance_reach
        self.feet = {leg: robot.foot(leg, self.q[leg]) for leg in robot.legs}
        self.lift_off = {leg: f.copy() for leg, f in self.feet.items()}

    # -- planning helpers ----------------------------------------------------------

    def _foot_z(self) -> float:
        """Foot-sphere centre height in base_link for the current body height."""
        return self.robot.foot_radius - self.height

    def lying_foot_z(self) -> float:
        p = self.params
        return -p.lying_height + self.robot.foot_radius + p.foot_clearance

    def neutral(self, leg: str, reach: float | None = None, z: float | None = None) -> np.ndarray:
        m = self.robot.mount(leg)
        r = m.xyz[:2] + (self.reach if reach is None else reach) * np.array([math.cos(m.yaw), math.sin(m.yaw)])
        return np.array([r[0], r[1], self._foot_z() if z is None else z])

    def _foot_velocity(self, p: np.ndarray, cmd: np.ndarray) -> np.ndarray:
        vx, vy, wz = cmd
        return np.array([vx - wz * p[1], vy + wz * p[0], 0.0])

    def _limit(self, cmd: np.ndarray) -> np.ndarray:
        """Clip the command to the speed limits and to what fits in one step."""
        p = self.params
        cmd = np.array(cmd, dtype=float)
        lin = math.hypot(cmd[0], cmd[1])
        if lin > p.max_linear:
            cmd[:2] *= p.max_linear / lin
        cmd[2] = max(-p.max_angular, min(p.max_angular, cmd[2]))
        t_stance = p.period / 2
        stroke = max(np.linalg.norm(self._foot_velocity(self.neutral(l), cmd)) * t_stance for l in self.robot.legs)
        if stroke > p.step_length:
            cmd *= p.step_length / stroke
        return cmd

    def _swing_group(self) -> int:
        return 0 if self.phase < 0.5 else 1

    def _settled(self) -> bool:
        return all(np.linalg.norm(self.feet[l][:2] - self.neutral(l)[:2]) < self.params.settle_tolerance
                   for l in self.robot.legs)

    # -- public API ----------------------------------------------------------------

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        self.cmd = np.array([vx, vy, wz], dtype=float)

    def stand_up(self) -> tuple[bool, str]:
        if self.mode is not Mode.LYING:
            return False, f"cannot stand up while {self.mode.value}"
        p = self.params
        z0 = self.lying_foot_z()
        tuck = {leg: self.robot.solve(leg, self.neutral(leg, p.rise_reach, z0))[0] for leg in self.robot.legs}
        self.segments = [_Segment(p.tuck_time, joints=tuck),
                         _Segment(p.rise_time, feet_z=(z0, self.robot.foot_radius - p.body_height))]
        self.mode = Mode.RISING
        return True, "standing up"

    def lie_down(self) -> tuple[bool, str]:
        if self.mode is not Mode.STANDING:
            return False, f"cannot lie down while {self.mode.value}"
        self.mode = Mode.WIDENING
        return True, "lying down"

    def step(self, dt: float) -> dict[str, np.ndarray]:
        """Advance by dt seconds and return joint angles per leg."""
        if self.segments:
            self._run_segment(dt)
            return dict(self.q)

        p = self.params
        if self.mode is Mode.LYING:
            return dict(self.q)
        if self.mode is Mode.RISING:
            self.mode = Mode.STANDING
        if self.mode is Mode.LOWERING:
            self.mode = Mode.LYING
            return dict(self.q)

        widening = self.mode is Mode.WIDENING
        self.reach = p.rise_reach if widening else p.stance_reach
        dh = max(-p.max_height_rate * dt, min(p.max_height_rate * dt, p.body_height - self.height))
        self.height += dh
        cmd = np.zeros(3) if widening else self._limit(self.cmd)
        q = self._walk(dt, cmd)
        if widening and not self.walking:
            self.segments = [_Segment(p.rise_time, feet_z=(self._foot_z(), self.lying_foot_z())),
                             _Segment(p.tuck_time, joints=self.rest)]
            self.mode = Mode.LOWERING
        return q

    # -- internals -----------------------------------------------------------------

    def _run_segment(self, dt: float) -> None:
        seg = self.segments[0]
        if seg.start is None:
            seg.start = {leg: self.q[leg].copy() for leg in self.robot.legs}
            if seg.feet_z is not None:
                self.feet = {leg: self.robot.foot(leg, self.q[leg]) for leg in self.robot.legs}
        seg.t += dt
        s = _smooth(seg.t / seg.duration)
        if seg.joints is not None:
            for leg in self.robot.legs:
                self.q[leg] = (1 - s) * seg.start[leg] + s * seg.joints[leg]
        else:
            z = (1 - s) * seg.feet_z[0] + s * seg.feet_z[1]
            for leg in self.robot.legs:
                self.feet[leg][2] = z
                self.q[leg] = self.robot.solve(leg, self.feet[leg])[0]
            self.height = self.robot.foot_radius - z
        if seg.done():
            self.segments.pop(0)
            self.feet = {leg: self.robot.foot(leg, self.q[leg]) for leg in self.robot.legs}

    def _walk(self, dt: float, cmd: np.ndarray) -> dict[str, np.ndarray]:
        p = self.params
        moving = np.any(np.abs(cmd) > 1e-6)

        if not self.walking:
            if not moving and self._settled():
                for leg in self.robot.legs:
                    self.feet[leg][2] = self._foot_z()
                return self._solve()
            self.walking, self.phase = True, 0.0
            self.lift_off = {l: self.feet[l].copy() for l in self.robot.legs}

        old_group = self._swing_group()
        self.phase += dt / p.period
        boundary = self.phase >= 0.5 * (old_group + 1)
        self.phase %= 1.0

        t_stance = p.period / 2
        for leg in TRIPODS[1 - old_group]:
            f = self.feet[leg]
            f -= self._foot_velocity(f, cmd) * dt
            f[2] = self._foot_z()
        for leg in TRIPODS[old_group]:
            n = self.neutral(leg)
            touchdown = n + self._foot_velocity(n, cmd) * (t_stance / 2)
            s = 1.0 if boundary else (self.phase % 0.5) / 0.5
            start = self.lift_off[leg]
            xy = start[:2] + (touchdown[:2] - start[:2]) * _smooth(s)
            self.feet[leg] = np.array([xy[0], xy[1], self._foot_z() + p.lift_height * math.sin(math.pi * s)])

        if boundary:
            for leg in TRIPODS[1 - old_group]:
                self.lift_off[leg] = self.feet[leg].copy()
            if not moving and self._settled():
                self.walking, self.phase = False, 0.0
        return self._solve()

    def _solve(self) -> dict[str, np.ndarray]:
        for leg in self.robot.legs:
            self.q[leg] = self.robot.solve(leg, self.feet[leg])[0]
        return dict(self.q)


class JointRateLimiter:
    """Clamps per-tick joint motion to the servo speed and reports the resulting velocity."""

    def __init__(self, max_velocity: float):
        self.max_velocity = max_velocity
        self.last: np.ndarray | None = None

    def __call__(self, q: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        q = np.asarray(q, dtype=float)
        if self.last is None or dt <= 0.0:
            self.last = q
            return q, np.zeros_like(q)
        step = np.clip(q - self.last, -self.max_velocity * dt, self.max_velocity * dt)
        q = self.last + step
        self.last = q
        return q, step / dt
