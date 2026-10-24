"""Joint angles (radians, conventions of legs.yaml) <-> STS3215 ticks, per servos.yaml."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .sts import TICKS_PER_TURN

TICKS_PER_RAD = TICKS_PER_TURN / (2 * math.pi)


@dataclass
class ServoCal:
    joint: str
    id: int
    zero: int        # ticks at joint angle 0
    sign: int        # +1 if servo ticks grow with the joint angle, else -1


class ServoMap:
    def __init__(self, path: str | Path, joint_names: list[str]):
        self.path = Path(path)
        with open(self.path) as f:
            raw = yaml.safe_load(f)["servos"]
        missing = [j for j in joint_names if j not in raw]
        if missing:
            raise ValueError(f"{self.path}: no servo for {missing}")
        self.cal = [ServoCal(j, int(raw[j]["id"]), int(raw[j]["zero"]), int(raw[j]["sign"])) for j in joint_names]
        self.ids = [c.id for c in self.cal]
        self._index = {c.joint: i for i, c in enumerate(self.cal)}

    def index(self, joint: str) -> int:
        return self._index[joint]

    def to_ticks(self, q) -> list[int]:
        return [int(round(c.zero + c.sign * a * TICKS_PER_RAD)) for c, a in zip(self.cal, q)]

    def to_angles(self, ticks: dict[int, int]) -> np.ndarray | None:
        """Joint angles from present positions; None if any servo did not answer."""
        if any(c.id not in ticks for c in self.cal):
            return None
        return np.array([c.sign * (ticks[c.id] - c.zero) / TICKS_PER_RAD for c in self.cal])

    def speeds(self, v) -> list[int]:
        return [int(round(abs(x) * TICKS_PER_RAD)) for x in v]

    def check_range(self, limits: list[tuple[float, float]]) -> list[str]:
        """Joints whose limit range would leave 0..4095 (servo single-turn range)."""
        bad = []
        for c, (lo, hi) in zip(self.cal, limits):
            ends = [c.zero + c.sign * a * TICKS_PER_RAD for a in (lo, hi)]
            if min(ends) < 0 or max(ends) > TICKS_PER_TURN - 1:
                bad.append(c.joint)
        return bad

    def save_zeros(self, zeros: dict[str, int]) -> None:
        with open(self.path) as f:
            data = yaml.safe_load(f)
        for joint, ticks in zeros.items():
            data["servos"][joint]["zero"] = int(ticks)
        with open(self.path, "w") as f:
            f.write(HEADER)
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)


HEADER = """\
# Servo bus id, encoder ticks at joint zero, and direction for every joint.
# Joint zero and positive direction are defined in spider_description/config/legs.yaml.
# Ids: <side><leg><joint>, side 1 = left, 2 = right; leg 1 = front, 2 = middle, 3 = rear;
# joint 1 = coxa, 2 = femur, 3 = tibia (numbering from numerate_motors.ipynb).
# `python3 -m spider_pi.calibrate zero` writes each servo's EEPROM offset so the zero pose
# reads 2048 (as in leg_twister.ipynb); `zero` stays 2048 then. Check `sign` with `... jog`.
"""


class FakeBus:
    """In-memory STS bus: servos jump to their goals. For tests and dry runs without hardware."""

    def __init__(self, ids: list[int], start: dict[int, int] | None = None):
        self.pos = {i: 2048 for i in ids}
        self.pos.update(start or {})
        self.torque_on = {i: False for i in ids}
        self.goals: list[dict[int, tuple[int, int]]] = []

    def ping(self, servo_id: int) -> bool:
        return servo_id in self.pos

    def torque(self, ids, enable) -> None:
        for i in ids:
            self.torque_on[i] = enable

    def positions(self, ids) -> dict[int, int]:
        return {i: self.pos[i] for i in ids if i in self.pos}

    def move(self, goals) -> None:
        self.goals.append(dict(goals))
        for i, (p, _) in goals.items():
            if self.torque_on.get(i):
                self.pos[i] = p

    def center_here(self, servo_id: int) -> int:
        offset = self.pos[servo_id] - 2048
        self.pos[servo_id] = 2048
        return offset

    def close(self) -> None:
        pass
