import math
from pathlib import Path

import numpy as np
import pytest

from spider_gait.kinematics import Hexapod

CONFIG = Path(__file__).resolve().parents[2] / "spider_description" / "config" / "legs.yaml"


@pytest.fixture(scope="module")
def robot() -> Hexapod:
    return Hexapod.load(CONFIG)


def workspace(robot: Hexapod, n: int, seed: int = 0):
    """Random joint angles inside the limits, on the knee-up branch, with the foot outside the coxa axis."""
    rng = np.random.default_rng(seed)
    lo = [robot.limits[j][0] for j in robot.joints]
    hi = [robot.limits[j][1] for j in robot.joints]
    g = robot.geometry
    out = []
    while len(out) < n:
        q = rng.uniform(lo, hi)
        knee = q[2] + g.beta - g.alpha
        reach = g.a + g.l1 * math.cos(q[1] + g.alpha) + g.l2 * math.cos(q[1] + q[2] + g.beta)
        if -math.pi + 0.05 < knee < -0.05 and reach > 0.03:
            out.append(q)
    return out


def test_ik_inverts_fk(robot):
    for leg in robot.legs:
        for q in workspace(robot, 500):
            p = robot.foot(leg, q)
            q2, ok = robot.solve(leg, p)
            assert ok
            np.testing.assert_allclose(q2, q, atol=1e-9)


def test_fk_inverts_ik(robot):
    rng = np.random.default_rng(1)
    for leg in robot.legs:
        m = robot.mount(leg)
        for _ in range(500):
            p_leg = np.array([rng.uniform(0.08, 0.22), rng.uniform(-0.08, 0.08), rng.uniform(-0.16, -0.02)])
            p = m.to_body(p_leg)
            q, ok = robot.solve(leg, p)
            if ok:
                np.testing.assert_allclose(robot.foot(leg, q), p, atol=1e-9)


def test_zero_pose(robot):
    g = robot.geometry
    p = g.forward([0.0, 0.0, 0.0])
    # femur horizontal, tibia straight down: foot below the knee
    assert p[0] == pytest.approx(g.a + g.l1 * math.cos(g.alpha), abs=1e-3)
    assert p[2] < g.h - 0.1


def test_joint_signs(robot):
    g = robot.geometry
    q = np.array([0.0, 0.3, -0.6])
    base = g.forward(q)
    assert g.forward(q + [0.1, 0, 0])[1] > base[1]      # coxa + turns the leg counter-clockwise
    assert g.forward(q + [0, 0.1, 0])[2] > base[2]      # femur + lifts the foot
    assert g.forward(q + [0, 0, 0.1])[0] > base[0]      # tibia + swings the foot outward


def test_mounts_are_mirrored(robot):
    left = [robot.mount(n) for n in ("lf", "lm", "lr")]
    right = [robot.mount(n) for n in ("rf", "rm", "rr")]
    for l, r in zip(left, right):
        assert l.xyz[0] == pytest.approx(r.xyz[0], abs=1e-3)
        assert l.xyz[1] == pytest.approx(-r.xyz[1], abs=1e-3)
        assert l.yaw == pytest.approx(-r.yaw, abs=2e-3)


def test_unreachable_is_flagged(robot):
    _, ok = robot.solve("lf", [1.0, 1.0, 0.0])
    assert not ok
