"""Pi runtime end to end on the in-memory bus, driven over real UDP like pi_bridge does."""
import socket
import threading
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from spider_pi import protocol
from spider_pi.robot import Robot, load_config
from spider_pi.servos import FakeBus, ServoMap

CONFIG = Path(__file__).resolve().parents[1] / "config.yaml"


@pytest.fixture
def setup():
    cfg = load_config(CONFIG)
    cfg["gait"].update(tuck_time=0.3, rise_time=0.3)
    cfg["startup_velocity"] = 4.0
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    server.setblocking(False)
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(1.0)
    ids = [c["id"] for c in yaml.safe_load(open(cfg["servos"]))["servos"].values()]
    bus = FakeBus(ids, start={i: 2048 + 300 for i in ids})     # not in rest pose at power-up
    robot = Robot(cfg, bus, server)
    stop = threading.Event()
    robot.start()
    t = threading.Thread(target=robot.run, args=(stop.is_set,), daemon=True)
    t.start()
    addr = server.getsockname()
    yield robot, bus, client, addr
    stop.set()
    t.join(2)
    robot.stop()
    server.close()
    client.close()


def call(client, addr, msg, n=[0]):
    n[0] += 1
    client.sendto(protocol.encode(dict(msg, id=n[0])), addr)
    deadline = time.time() + 2
    while time.time() < deadline:
        m = protocol.decode(client.recv(65536))
        if m["type"] == "reply" and m["id"] == n[0]:
            return m
    raise TimeoutError


def last_state(client):
    while True:
        m = protocol.decode(client.recv(65536))
        if m["type"] == "state":
            return m


def wait_mode(robot, mode, seconds):
    deadline = time.time() + seconds
    while time.time() < deadline and robot.gait.mode.value != mode:
        time.sleep(0.05)
    return robot.gait.mode.value


def test_full_cycle(setup):
    robot, bus, client, addr = setup
    client.sendto(protocol.encode({"type": "cmd", "vx": 0, "vy": 0, "wz": 0}), addr)
    time.sleep(1.0)
    assert not robot.starting, "should reach the rest pose"
    servos = ServoMap(robot.cfg["servos"], robot.names)
    rest = np.tile(robot.hexapod.rest_pose, 6)
    np.testing.assert_allclose(servos.to_angles(bus.pos), rest, atol=2e-3)

    assert call(client, addr, {"type": "stand_up"})["ok"]
    assert wait_mode(robot, "standing", 3) == "standing"
    assert not call(client, addr, {"type": "stand_up"})["ok"]

    for _ in range(20):                                  # walk 1 s, cmd at 20 Hz
        client.sendto(protocol.encode({"type": "cmd", "vx": 0.05, "vy": 0, "wz": 0}), addr)
        time.sleep(0.05)
    assert robot.gait.walking
    time.sleep(robot.cfg["cmd_timeout"] + 2 * robot.gait.params.period + 0.3)
    assert not robot.gait.walking, "must stop after the command times out"

    st = last_state(client)
    assert st["mode"] == "standing" and len(st["position"]) == 18

    assert call(client, addr, {"type": "set", "params": {"period": 0.6, "bogus": 1}})["ok"]
    assert robot.gait.params.period == 0.6

    assert call(client, addr, {"type": "lie_down"})["ok"]
    assert wait_mode(robot, "lying", 5) == "lying"


def test_goals_respect_limits_and_speed(setup):
    robot, bus, client, addr = setup
    time.sleep(0.5)
    servos = ServoMap(robot.cfg["servos"], robot.names)
    for goals in bus.goals:
        ticks = {i: p for i, (p, _) in goals.items()}
        q = servos.to_angles(ticks)
        assert np.all(q >= robot.lo - 1e-3) and np.all(q <= robot.hi + 1e-3)
        assert all(s > 0 for _, s in goals.values())     # 0 would mean "full speed" on STS


def test_shutdown_lies_down_and_releases_torque(setup):
    robot, bus, client, addr = setup
    time.sleep(1.0)
    call(client, addr, {"type": "stand_up"})
    assert wait_mode(robot, "standing", 3) == "standing"
    robot.stop()
    assert robot.gait.mode.value == "lying"
    assert not any(bus.torque_on.values())
