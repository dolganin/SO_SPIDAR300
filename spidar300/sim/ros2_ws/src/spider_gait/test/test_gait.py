from pathlib import Path

import numpy as np
import pytest

from spider_gait.gait import TRIPODS, GaitParams, JointRateLimiter, Mode, TripodGait
from spider_gait.kinematics import Hexapod

CONFIG = Path(__file__).resolve().parents[2] / "spider_description" / "config" / "legs.yaml"
DT = 0.02


@pytest.fixture
def gait() -> TripodGait:
    g = TripodGait(Hexapod.load(CONFIG), GaitParams(tuck_time=0.5, rise_time=0.5))
    assert g.stand_up()[0]
    run(g, 5.0)
    assert g.mode is Mode.STANDING and not g.walking
    return g


def run(gait, seconds, cmd=(0.0, 0.0, 0.0)):
    gait.set_command(*cmd)
    out = []
    for _ in range(int(seconds / DT)):
        out.append(gait.step(DT))
    return out


def test_zero_command_stands_still(gait):
    q0 = gait.step(DT)
    for q in run(gait, 2.0):
        for leg in q:
            np.testing.assert_allclose(q[leg], q0[leg], atol=1e-9)
    assert not gait.walking


def test_walk_stays_reachable_and_in_limits(gait):
    robot = gait.robot
    for cmd in [(0.1, 0, 0), (0, 0.1, 0), (0, 0, 0.6), (0.07, 0.07, 0.4)]:
        run(gait, 0.1, cmd)
        for _ in range(int(3.0 / DT)):
            gait.step(DT)
            for leg in robot.legs:
                _, ok = robot.solve(leg, gait.feet[leg])
                assert ok, (cmd, leg, gait.feet[leg])


def test_stance_feet_move_against_command(gait):
    run(gait, 1.0, (0.05, 0.0, 0.0))
    swing = 0 if gait.phase < 0.5 else 1
    stance = TRIPODS[1 - swing]
    before = {l: gait.feet[l].copy() for l in stance}
    gait.step(DT)
    if (0 if gait.phase < 0.5 else 1) == swing:
        for l in stance:
            np.testing.assert_allclose(gait.feet[l] - before[l], [-0.05 * DT, 0, 0], atol=1e-9)


def test_stops_in_stance_after_command_ends(gait):
    run(gait, 2.0, (0.08, 0.0, 0.3))
    run(gait, 3.0)
    assert not gait.walking
    for leg in gait.robot.legs:
        np.testing.assert_allclose(gait.feet[leg], gait.neutral(leg), atol=gait.params.settle_tolerance)


def test_command_is_limited(gait):
    p = gait.params
    cmd = gait._limit([1.0, 0.0, 5.0])
    assert np.hypot(cmd[0], cmd[1]) <= p.max_linear + 1e-12
    assert abs(cmd[2]) <= p.max_angular + 1e-12


def test_rate_limiter():
    lim = JointRateLimiter(1.0)
    lim(np.zeros(2), DT)
    q, v = lim(np.array([1.0, -0.001]), DT)
    np.testing.assert_allclose(q, [DT, -0.001])
    np.testing.assert_allclose(v, [1.0, -0.05])


def test_starts_lying_in_rest_pose():
    g = TripodGait(Hexapod.load(CONFIG))
    assert g.mode is Mode.LYING
    g.set_command(0.1, 0, 0)
    for q in run(g, 1.0, (0.1, 0, 0)):
        for leg in q:
            np.testing.assert_allclose(q[leg], g.robot.rest_pose)
    assert not g.lie_down()[0]


def test_stand_up_presses_feet_straight_down(gait):
    g = TripodGait(gait.robot, GaitParams(tuck_time=0.5, rise_time=0.5))
    g.stand_up()
    run(g, 0.5)                                  # tuck finished
    xy = {l: g.feet[l][:2].copy() for l in g.robot.legs}
    for _ in range(int(0.5 / DT)):
        g.step(DT)
        for l in g.robot.legs:
            np.testing.assert_allclose(g.feet[l][:2], xy[l], atol=1e-9)
            assert g.robot.solve(l, g.feet[l])[1]
    assert g.height == pytest.approx(g.params.body_height)


def test_lie_down_returns_to_rest(gait):
    run(gait, 1.0, (0.05, 0, 0))
    assert gait.lie_down()[0]
    run(gait, 6.0, (0.05, 0, 0))                 # walking commands are ignored meanwhile
    assert gait.mode is Mode.LYING
    for leg in gait.robot.legs:
        np.testing.assert_allclose(gait.q[leg], gait.robot.rest_pose, atol=1e-9)
    assert gait.stand_up()[0]
