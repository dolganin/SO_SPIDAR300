"""Raspberry Pi main loop: UDP commands -> tripod gait -> STS3215 servos.

  python3 -m spider_pi.robot [--config config.yaml] [--fake]

Starts waiting for support approval, moves to configured zero then rest_pose,
and waits for floor approval. Startup does not write servo EEPROM.
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
from .servos import FakeBus, ServoMap, TICKS_PER_RAD

log = logging.getLogger("spider_pi")
_HERE = Path(__file__).resolve().parents[1]


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
        self.starting = True
        self.startup = "waiting_support"
        self.startup_deadline = None
        self.next_startup_check = 0.0
        self.started = False
        self.client = None
        self.last_cmd = 0.0
        self.q = np.zeros(len(self.names))
        self.v = np.zeros(len(self.names))
        self.measured: np.ndarray | None = None
        self.ticks: dict[int, int] = {}            # latest present position per servo id
        self.read_cursor = 0
        limits = [self.hexapod.limits[j] for _ in self.hexapod.legs for j in self.hexapod.joints]
        self.lo, self.hi = np.array([l for l, _ in limits]), np.array([h for _, h in limits])
        # Intersect geometric limits with the encoder's single-turn range.
        for i, c in enumerate(self.servos.cal):
            ends = sorted(((0 - c.zero) / (c.sign * TICKS_PER_RAD),
                           (4095 - c.zero) / (c.sign * TICKS_PER_RAD)))
            self.lo[i] = max(self.lo[i], ends[0])
            self.hi[i] = min(self.hi[i], ends[1])
        bad = self.servos.check_range(limits)
        if bad:
            log.warning("joint limits leave the 0..4095 servo range (check zero/sign): %s", bad)

    # -- startup / shutdown ----------------------------------------------------------

    def start(self) -> None:
        missing = [i for i in self.servos.ids if not self.bus.ping(i)]
        if missing:
            raise RuntimeError(f"servos not answering: {missing}")
        self.ticks = self.bus.positions(self.servos.ids)
        measured = self.servos.to_angles(self.ticks)
        if measured is None:
            missing = [c.joint for c in self.servos.cal if c.id not in self.ticks]
            raise RuntimeError(f"could not read servo positions: {missing}")
        self.limiter.last = measured
        self.measured = measured
        self.q = measured.copy()
        log.info("Place the robot belly-down on a high support, feet clear. Press Enter in keyboard_teleop.")

    def approve_startup(self) -> tuple[bool, str]:
        if self.startup == "waiting_support":
            ticks = self.bus.positions(self.servos.ids)
            measured = self.servos.to_angles(ticks)
            if measured is None:
                return False, "cannot read all servos; startup not approved"
            goals = self.servos.to_ticks(np.tile(self.hexapod.rest_pose, len(self.hexapod.legs)))
            if any(t < 0 or t > 4095 for t in goals):
                return False, "lying pose outside encoder range; check zero/sign"
            # Set present positions as goals before enabling torque: no stale-goal jump.
            self.bus.move({i: (ticks[i], int(self.cfg["bus"]["goal_speed"])) for i in self.servos.ids})
            self.bus.torque(self.servos.ids, True)
            self.started = True
            self.limiter.last = measured.copy()
            self.startup = "moving_default"
            self.startup_deadline = time.monotonic() + float(self.cfg.get("startup_timeout", 20.0))
            return True, "moving to configured default (geometric zero)"
        if self.startup == "waiting_floor":
            self.startup = "ready"
            self.starting = False
            self.limiter.max_velocity = self.hexapod.max_velocity
            self.gait.set_command(0, 0, 0)
            return True, "floor placement confirmed; ready to stand up"
        return False, f"cannot approve while {self.startup}"

    def stop(self) -> None:
        if not self.started:
            return
        if self.startup == "ready" and self.gait.mode is not Mode.LYING:
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
            if kind == "approve":
                ok, text = self.approve_startup()
                log.info(text)
                self.reply(msg, ok, text)
            elif kind == "cmd":
                if self.starting:
                    continue
                self.gait.set_command(float(msg.get("vx", 0)), float(msg.get("vy", 0)), float(msg.get("wz", 0)))
                self.last_cmd = now
            elif kind in ("stand_up", "lie_down"):
                if self.starting:
                    ok, text = False, f"startup not complete: {self.startup}"
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
        if self.startup in ("waiting_support", "waiting_floor", "fault"):
            return
        if self.startup in ("moving_default", "lifting_legs"):
            target = (np.zeros(len(self.names)) if self.startup == "moving_default"
                      else np.tile(self.hexapod.rest_pose, len(self.hexapod.legs)))
        else:
            q = self.gait.step(dt)
            target = np.concatenate([q[leg] for leg in self.hexapod.legs])
        self.q, self.v = self.limiter(np.clip(target, self.lo, self.hi), dt)
        ticks = self.servos.to_ticks(self.q)
        # Fixed goal speed, as in spider_control.py: the rate limiter above already moves the goal
        # smoothly every tick, and a speed tied to the joint velocity left lagging servos creeping
        # towards their last goal at the minimum speed long after the motion had ended.
        speed = int(self.cfg["bus"].get("goal_speed", 2000))
        self.bus.move({i: (t, speed) for i, t in zip(self.servos.ids, ticks)})
        if self.startup in ("moving_default", "lifting_legs"):
            if time.monotonic() < self.next_startup_check:
                return
            self.next_startup_check = time.monotonic() + 0.1
            measured = self.servos.to_angles(self.bus.positions(self.servos.ids))
            reached = (measured is not None and np.max(np.abs(measured - target)) < 0.025
                       and np.max(np.abs(self.q - target)) < 1e-3)
            if reached:
                if self.startup == "moving_default":
                    self.startup = "lifting_legs"
                    self.startup_deadline = time.monotonic() + float(self.cfg.get("startup_timeout", 20.0))
                    log.info("default reached; lifting legs into lying pose")
                else:
                    self.startup = "waiting_floor"
                    log.info("lying pose reached; holding. Place belly-down on the floor, then press Enter.")
            elif time.monotonic() > self.startup_deadline:
                # Freeze at measured positions; never authorize walking after a failed move.
                present = self.bus.positions(self.servos.ids)
                self.bus.move({i: (p, speed) for i, p in present.items()})
                self.startup = "fault"
                log.error("startup move timed out; holding, walking blocked; restart after inspection")

    def read_some(self) -> None:
        """Read a few servos per tick (round robin) so the bus never stalls the 50 Hz loop."""
        n = int(self.cfg["bus"].get("reads_per_tick", 3))
        ids = self.servos.ids
        batch = [ids[(self.read_cursor + k) % len(ids)] for k in range(min(n, len(ids)))]
        self.read_cursor = (self.read_cursor + len(batch)) % len(ids)
        self.ticks.update(self.bus.positions(batch))
        measured = self.servos.to_angles(self.ticks)
        if measured is not None:
            self.measured = measured

    def publish_state(self) -> None:
        self.send({"type": "state", "mode": self.gait.mode.value if not self.starting else self.startup, "names": self.names,
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
            current_mode = self.startup if self.starting else self.gait.mode.value
            if current_mode != mode:
                mode = current_mode
                log.info("mode: %s", mode)
            self.read_some()
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
