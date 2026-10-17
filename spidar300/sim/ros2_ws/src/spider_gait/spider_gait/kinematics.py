"""Hexapod leg kinematics, independent of ROS.

Everything comes from spider_description/config/legs.yaml. Each leg is the same
coxa-femur-tibia chain mounted on base_link at `xyz` and rotated by `yaw`:

  coxa   rotation about +z of the mount frame, 0 = leg along mount +x
  femur  rotation about -y, 0 = femur horizontal, + = knee up
  tibia  rotation about -y, 0 = tibia vertical when femur is 0, + = foot outward

Within the leg plane every rotation about -y turns a vector (x, z) by +q towards +z.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


def joint_name(leg: str, joint: str) -> str:
    return f"leg_{leg}_{joint}"


@dataclass(frozen=True)
class LegGeometry:
    """Leg chain in the coxa frame: femur axis at (a, b, h), femur and foot vectors."""
    a: float
    b: float
    h: float
    l1: float
    alpha: float      # femur vector angle in the leg plane at q = 0
    l2: float
    beta: float       # foot vector angle in the leg plane at q = 0
    w: float          # lateral offset of the foot from the coxa axis, constant

    @classmethod
    def from_config(cls, geometry: dict) -> LegGeometry:
        cx, cy, cz = geometry["coxa"]
        fx, fy, fz = geometry["femur"]
        tx, ty, tz = geometry["foot"]
        return cls(a=cx, b=cy, h=cz,
                   l1=math.hypot(fx, fz), alpha=math.atan2(fz, fx),
                   l2=math.hypot(tx, tz), beta=math.atan2(tz, tx),
                   w=cy + fy + ty)

    def forward(self, q) -> np.ndarray:
        """Foot position in the mount frame for joint angles (coxa, femur, tibia)."""
        qc, qf, qt = q
        r = self.a + self.l1 * math.cos(qf + self.alpha) + self.l2 * math.cos(qf + qt + self.beta)
        z = self.h + self.l1 * math.sin(qf + self.alpha) + self.l2 * math.sin(qf + qt + self.beta)
        c, s = math.cos(qc), math.sin(qc)
        return np.array([r * c - self.w * s, r * s + self.w * c, z])

    def inverse(self, p) -> tuple[np.ndarray, bool]:
        """Knee-up joint angles reaching mount-frame point p; False if p was out of reach."""
        x, y, z = p
        reachable = True
        rho2 = x * x + y * y
        if rho2 < self.w * self.w:
            reachable = False
            rho2 = self.w * self.w
        r = math.sqrt(rho2 - self.w * self.w)
        qc = math.atan2(y, x) - math.atan2(self.w, r)

        dx, dz = r - self.a, z - self.h
        cos_g = (dx * dx + dz * dz - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        if abs(cos_g) > 1.0:
            reachable = False
            cos_g = max(-1.0, min(1.0, cos_g))
        g = -math.acos(cos_g)
        qf = math.atan2(dz, dx) - math.atan2(self.l2 * math.sin(g), self.l1 + self.l2 * math.cos(g)) - self.alpha
        qt = g + self.alpha - self.beta
        return np.array([math.remainder(qc, math.tau), qf, qt]), reachable


@dataclass(frozen=True)
class Mount:
    """Where a leg's coxa axis sits on base_link and which way its zero points."""
    name: str
    xyz: np.ndarray
    yaw: float

    def to_leg(self, p_body) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        d = np.asarray(p_body, dtype=float) - self.xyz
        return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])

    def to_body(self, p_leg) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y, z = p_leg
        return self.xyz + np.array([c * x - s * y, s * x + c * y, z])


class Hexapod:
    """Mount table plus the shared leg chain; foot points are in base_link."""

    def __init__(self, config: dict):
        self.joints: list[str] = list(config["joints"])
        self.mounts: list[Mount] = [Mount(l["name"], np.array(l["xyz"], dtype=float), float(l["yaw"]))
                                    for l in config["legs"]]
        self.legs: list[str] = [m.name for m in self.mounts]
        self.geometry = LegGeometry.from_config(config["geometry"])
        self.limits = {j: tuple(config["limits"][j]) for j in self.joints}
        self.max_velocity = float(config["servo"]["velocity"])
        self.foot_radius = float(config["geometry"].get("foot_radius", 0.0))
        rest = config.get("rest_pose", {})
        self.rest_pose = np.array([float(rest.get(j, 0.0)) for j in self.joints])

    @classmethod
    def load(cls, path: str | Path) -> Hexapod:
        with open(path) as f:
            return cls(yaml.safe_load(f))

    @property
    def joint_names(self) -> list[str]:
        return [joint_name(leg, j) for leg in self.legs for j in self.joints]

    def mount(self, leg: str) -> Mount:
        return self.mounts[self.legs.index(leg)]

    def foot(self, leg: str, q) -> np.ndarray:
        m = self.mount(leg)
        return m.to_body(self.geometry.forward(q))

    def solve(self, leg: str, p_body) -> tuple[np.ndarray, bool]:
        """Joint angles for a foot point in base_link; False if unreachable or past a joint limit."""
        q, ok = self.geometry.inverse(self.mount(leg).to_leg(p_body))
        for i, j in enumerate(self.joints):
            lo, hi = self.limits[j]
            if not lo <= q[i] <= hi:
                ok = False
                q[i] = min(hi, max(lo, q[i]))
        return q, ok
