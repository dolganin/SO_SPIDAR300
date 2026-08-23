from __future__ import annotations

import numpy as np

from .. import kinematics as K

class TripodGait:

    GROUP_A = ("lf", "rm", "lr")
    GROUP_B = ("rf", "lm", "rr")

    def __init__(
        self,
        cycle_time: float = 0.8,
        stride: float = 0.06,
        step_height: float = 0.04,
        duty: float = 0.5,
    ):
        self.cycle_time = cycle_time
        self.stride = stride
        self.step_height = step_height
        self.duty = duty
        self.phase = 0.0
        self._nominal = {leg.name: K.nominal_foot_body(leg) for leg in K.LEGS}

    def _stroke_dir(self, leg: K.LegDef, vx: float, vy: float, wz: float) -> np.ndarray:
        p = self._nominal[leg.name]
        sx = vx - wz * p[1]
        sy = vy + wz * p[0]
        return np.array([sx, sy]) * self.stride

    def _foot_target(self, leg: K.LegDef, leg_phase: float,
                     stroke: np.ndarray) -> np.ndarray:
        p0 = self._nominal[leg.name].copy()
        if leg_phase < self.duty:
            prog = leg_phase / self.duty
            p0[:2] += stroke * (0.5 - prog)
        else:
            prog = (leg_phase - self.duty) / (1.0 - self.duty)
            p0[:2] += stroke * (-0.5 + prog)
            p0[2] += self.step_height * np.sin(np.pi * prog)
        return p0

    def step(self, dt: float, vx: float = 0.0, vy: float = 0.0,
             wz: float = 0.0) -> np.ndarray:
        moving = abs(vx) + abs(vy) + abs(wz) > 1e-6
        if moving:
            self.phase = (self.phase + dt / self.cycle_time) % 1.0
        targets = self.joint_targets(vx, vy, wz)
        return targets

    def joint_targets(self, vx: float = 0.0, vy: float = 0.0,
                      wz: float = 0.0) -> np.ndarray:
        out = np.zeros(len(K.JOINT_NAMES_18))
        idx = 0
        for leg in K.LEGS:
            lp = self.phase if leg.name in self.GROUP_A else (self.phase + 0.5) % 1.0
            stroke = self._stroke_dir(leg, vx, vy, wz)
            foot = self._foot_target(leg, lp, stroke)
            try:
                c, f, t = K.ik_leg_body(leg, foot)
            except ValueError:
                c, f, t = K.default_stance_angles()
            out[idx] = K.clamp("coxa", c)
            out[idx + 1] = K.clamp("femur", f)
            out[idx + 2] = K.clamp("tibia", t)
            idx += 3
        return out

    def stance_targets(self) -> np.ndarray:
        return K.stance_joint_vector()
