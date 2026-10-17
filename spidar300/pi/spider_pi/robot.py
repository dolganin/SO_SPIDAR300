"""Raspberry Pi main loop: UDP commands -> tripod gait -> STS3215 servos.

  python3 -m spider_pi.robot [--config config.yaml] [--fake]

Starts by moving slowly from the measured pose into rest_pose (lying, legs spread).
Without a velocity command for cmd_timeout the gait stops; on Ctrl-C / SIGTERM it
lies down first, then switches the torque off.
"""
from __future__ import annotations

import argparse
import logging
import signal
import socket
import time
from dataclasses import fields
from pathlib import Path

import numpy as np
import yaml

from spider_gait.gait import GaitParams, JointRateLimiter, Mode, TripodGait
from spider_gait.kinematics import Hexapod

from . import protocol
from .servos import FakeBus, ServoMap

log = logging.getLogger("spider_pi")
_HERE = Path(__file__).resolve().parents[1]
MIN_SPEED_TICKS = 30          # STS speed 0 means "maximum"; never send it


def load_config(path: Path) -> dict:
    with open(path) as f:
        cfg = yaml.safe_load(f)
    for key in ("legs", "servos"):
        cfg[key] = str((path.parent / cfg[key]).resolve())
    return cfg


class Robot:
    def __init__(self, cfg: dict, bus, sock: socket.socket):
        self.cfg, self.bus, self.sock = cfg, bus, sock
        self.hexapod = Hexapod.load(cfg["legs"])
        self.names = self.hexapod.joint_names
        self.servos = ServoMap(cfg["servos"], self.names)
        params = GaitParams(**{k: float(v) for k, v in cfg.get("gait", {}).items()})
        self.gait = TripodGait(self.hexapod, params)
        self.limiter = JointRateLimiter(float(cfg["startup_velocity"]))
        self.starting = True          # slow first move into rest_pose
        self.started = False
        self.client = None
        self.last_cmd = 0.0
        self.q = np.zeros(len(self.names))
        self.v = np.zeros(len(self.names))
        self.measured: np.ndarray | None = None
        limits = [self.hexapod.limits[j] for _ in self.hexapod.legs for j in self.hexapod.joints]
        self.lo, self.hi = np.array([l for l, _ in limits]), np.array([h for _, h in limits])
        bad = self.servos.check_range(limits)
        if bad:
            log.warning("joint limits leave the 0..4095 servo range (check zero/sign): %s", bad)

    # -- startup / shutdown ----------------------------------------------------------

    def start(self) -> None:
        missing = [i for i in self.servos.ids if not self.bus.ping(i)]
        if missing:
            raise RuntimeError(f"servos not answering: {missing}")
        measured = self.servos.to_angles(self.bus.positions(self.servos.ids))
        if measured is None:
            raise RuntimeError("could not read servo positions")
        self.limiter.last = measured
        self.measured = measured
        self.bus.torque(self.servos.ids, True)
        self.started = True
        log.info("torque on; moving into rest pose at %.1f rad/s", self.limiter.max_velocity)

    def stop(self) -> None:
        if not self.started:
            return
        if self.gait.mode is not Mode.LYING:
            log.info("lying down before torque off")
            deadline = time.monotonic() + float(self.cfg["shutdown_timeout"])
            period = 1.0 / float(self.cfg["rate"])
            self.gait.set_command(0, 0, 0)
            while self.gait.mode is not Mode.LYING and time.monotonic() < deadline:
                if self.gait.mode is Mode.STANDING:
                    self.gait.lie_down()
                self.control(period)
                time.sleep(period)
        self.bus.torque(self.servos.ids, False)
        log.info("torque off")

    # -- UDP -------------------------------------------------------------------------

    def poll_udp(self, now: float) -> None:
        while True:
            try:
                data, addr = self.sock.recvfrom(protocol.MAX_DATAGRAM)
            except (BlockingIOError, InterruptedError):
                return
            msg = protocol.decode(data)
            if msg is None:
                continue
            if addr != self.client:
                log.info("client %s:%d", *addr)
                self.client = addr
            kind = msg["type"]
            if kind == "cmd":
                self.gait.set_command(float(msg.get("vx", 0)), float(msg.get("vy", 0)), float(msg.get("wz", 0)))
                self.last_cmd = now
            elif kind in ("stand_up", "lie_down"):
                if self.starting:
                    ok, text = False, "still moving into the rest pose"
                else:
                    ok, text = self.gait.stand_up() if kind == "stand_up" else self.gait.lie_down()
                log.info(text)
                self.reply(msg, ok, text)
            elif kind == "set":
                names = {f.name for f in fields(GaitParams)}
                updates = {k: float(v) for k, v in msg.get("params", {}).items() if k in names}
                for k, v in updates.items():
                    setattr(self.gait.params, k, v)
                log.info("gait parameters: %s", updates)
                self.reply(msg, True, f"set {sorted(updates)}")

    def reply(self, msg: dict, ok: bool, text: str) -> None:
        self.send({"type": "reply", "id": msg.get("id"), "ok": ok, "message": text})

    def send(self, msg: dict) -> None:
        if self.client is not None:
            try:
                self.sock.sendto(protocol.encode(msg), self.client)
            except OSError as e:
                log.debug("send failed: %s", e)

    # -- control ---------------------------------------------------------------------

    def control(self, dt: float) -> None:
        q = self.gait.step(dt)
        target = np.concatenate([q[leg] for leg in self.hexapod.legs])
        self.q, self.v = self.limiter(np.clip(target, self.lo, self.hi), dt)
        if self.starting and np.abs(self.q - target).max() < 1e-3:
            self.starting = False
            self.limiter.max_velocity = self.hexapod.max_velocity
            log.info("in rest pose; ready")
        ticks = self.servos.to_ticks(self.q)
        speeds = [max(MIN_SPEED_TICKS, int(s * 1.3)) for s in self.servos.speeds(self.v)]
        self.bus.move(dict(zip(self.servos.ids, zip(ticks, speeds))))

    def publish_state(self) -> None:
        measured = self.servos.to_angles(self.bus.positions(self.servos.ids))
        if measured is not None:
            self.measured = measured
        self.send({"type": "state", "mode": self.gait.mode.value, "names": self.names,
                   "position": [round(float(x), 4) for x in (self.measured if self.measured is not None else self.q)],
                   "command": [round(float(x), 4) for x in self.q]})

    def run(self, should_stop) -> None:
        period = 1.0 / float(self.cfg["rate"])
        state_every = max(1, int(round(float(self.cfg["rate"]) / float(self.cfg["state_rate"]))))
        timeout = float(self.cfg["cmd_timeout"])
        tick, next_t, last_t = 0, time.monotonic(), time.monotonic()
        mode = None
        while not should_stop():
            now = time.monotonic()
            self.poll_udp(now)
            if self.last_cmd and now - self.last_cmd > timeout:
                self.gait.set_command(0, 0, 0)
                self.last_cmd = 0.0
                log.info("no velocity command for %.1f s: stopping", timeout)
            self.control(min(0.1, now - last_t) if tick else period)
            last_t = now
            if self.gait.mode is not mode:
                mode = self.gait.mode
                log.info("mode: %s", mode.value)
            tick += 1
            if tick % state_every == 0:
                self.publish_state()
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="SPIDAR300 Raspberry Pi runtime")
    parser.add_argument("--config", type=Path, default=_HERE / "config.yaml")
    parser.add_argument("--fake", action="store_true", help="no hardware: in-memory servo bus")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    cfg = load_config(args.config.resolve())
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((cfg["udp"]["listen"], int(cfg["udp"]["port"])))
    sock.setblocking(False)

    if args.fake:
        ids = [c["id"] for c in yaml.safe_load(open(cfg["servos"]))["servos"].values()]
        bus = FakeBus(ids)
    else:
        from .sts import StsBus
        bus = StsBus(cfg["bus"]["port"], int(cfg["bus"]["baudrate"]))

    stop = {"flag": False}
    def on_signal(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    robot = Robot(cfg, bus, sock)
    log.info("listening on udp %s:%s", cfg["udp"]["listen"], cfg["udp"]["port"])
    try:
        robot.start()
        robot.run(lambda: stop["flag"])
    finally:
        try:
            robot.stop()
        finally:
            bus.close()
            sock.close()


if __name__ == "__main__":
    main()
