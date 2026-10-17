"""Servo setup and calibration.

  python3 -m spider_pi.calibrate scan                 list servo ids on the bus
  python3 -m spider_pi.calibrate set-id OLD NEW       renumber (only that servo connected)
  python3 -m spider_pi.calibrate zero                 torque off, pose by hand, record zeros
  python3 -m spider_pi.calibrate read                 live joint angles from the encoders
  python3 -m spider_pi.calibrate jog JOINT [DELTA]    move one joint by DELTA rad to check its sign

Zero pose (legs.yaml): every coxa points straight out from the body, every femur is
horizontal, every tibia hangs vertically. Hold the robot in the air.
Positive: coxa counter-clockwise seen from above, femur knee up, tibia foot outward.
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import yaml

from spider_gait.kinematics import Hexapod

from .robot import _HERE, load_config
from .servos import ServoMap
from .sts import StsBus


def open_bus(cfg) -> StsBus:
    return StsBus(cfg["bus"]["port"], int(cfg["bus"]["baudrate"]))


def cmd_scan(cfg, args) -> None:
    bus = open_bus(cfg)
    found = [i for i in range(args.first, args.last + 1) if bus.ping(i)]
    print(f"servos on {cfg['bus']['port']}: {found or 'none'}")
    expected = {c["id"] for c in yaml.safe_load(open(cfg["servos"]))["servos"].values()}
    if missing := sorted(expected - set(found)):
        print(f"missing from servos.yaml: {missing}")
    if extra := sorted(set(found) - expected):
        print(f"not in servos.yaml: {extra}")


def cmd_set_id(cfg, args) -> None:
    bus = open_bus(cfg)
    if not bus.ping(args.old):
        raise SystemExit(f"no servo with id {args.old}")
    bus.set_id(args.old, args.new)
    print(f"id {args.old} -> {args.new}: {'ok' if bus.ping(args.new) else 'FAILED'}")


def cmd_zero(cfg, args) -> None:
    names = Hexapod.load(cfg["legs"]).joint_names
    servos = ServoMap(cfg["servos"], names)
    bus = open_bus(cfg)
    bus.torque(servos.ids, False)
    print("Torque is off. Put the robot in the zero pose (see --help), hold it, press Enter.")
    input()
    ticks = bus.positions(servos.ids)
    missing = [c.joint for c in servos.cal if c.id not in ticks]
    if missing:
        raise SystemExit(f"no answer from {missing}; nothing saved")
    zeros = {c.joint: ticks[c.id] for c in servos.cal}
    for joint, t in zeros.items():
        print(f"  {joint:14s} id {servos.cal[names.index(joint)].id:3d}  zero {t}")
    servos.save_zeros(zeros)
    print(f"saved to {servos.path}")


def cmd_read(cfg, args) -> None:
    hexapod = Hexapod.load(cfg["legs"])
    servos = ServoMap(cfg["servos"], hexapod.joint_names)
    bus = open_bus(cfg)
    try:
        while True:
            q = servos.to_angles(bus.positions(servos.ids))
            if q is None:
                print("\rsome servos did not answer", end="")
            else:
                rows = [" ".join(f"{math.degrees(a):+6.1f}" for a in q[i:i + 3]) for i in range(0, len(q), 3)]
                print("\r" + " | ".join(f"{leg} {r}" for leg, r in zip(hexapod.legs, rows)), end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def cmd_jog(cfg, args) -> None:
    names = Hexapod.load(cfg["legs"]).joint_names
    if args.joint not in names:
        raise SystemExit(f"unknown joint {args.joint}; one of {names}")
    servos = ServoMap(cfg["servos"], names)
    cal = servos.cal[names.index(args.joint)]
    bus = open_bus(cfg)
    q = servos.to_angles(bus.positions(servos.ids))
    if q is None:
        raise SystemExit("could not read servo positions")
    i = names.index(args.joint)
    start = q[i]
    target = start + args.delta
    speed = int(0.5 * 4096 / (2 * math.pi))
    goal = servos.to_ticks([target] * len(names))[i]
    bus.torque([cal.id], True)
    bus.move({cal.id: (goal, speed)})
    time.sleep(abs(args.delta) / 0.5 + 0.5)
    expected = {"coxa": "counter-clockwise (seen from above)", "femur": "knee UP", "tibia": "foot OUTWARD"}
    part = args.joint.rsplit("_", 1)[1]
    print(f"{args.joint}: {math.degrees(start):+.1f} -> {math.degrees(target):+.1f} deg.")
    print(f"For a positive DELTA it should have moved {expected[part]}; if not, flip its sign in servos.yaml.")
    if not args.hold:
        bus.move({cal.id: (servos.to_ticks([start] * len(names))[i], speed)})
        time.sleep(abs(args.delta) / 0.5 + 0.5)
        bus.torque([cal.id], False)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=_HERE / "config.yaml")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("scan")
    p.add_argument("--first", type=int, default=1)
    p.add_argument("--last", type=int, default=30)
    p = sub.add_parser("set-id")
    p.add_argument("old", type=int)
    p.add_argument("new", type=int)
    sub.add_parser("zero")
    sub.add_parser("read")
    p = sub.add_parser("jog")
    p.add_argument("joint")
    p.add_argument("delta", type=float, nargs="?", default=0.3)
    p.add_argument("--hold", action="store_true", help="stay at the new angle with torque on")
    args = parser.parse_args(argv)
    cfg = load_config(args.config.resolve())
    {"scan": cmd_scan, "set-id": cmd_set_id, "zero": cmd_zero, "read": cmd_read, "jog": cmd_jog}[args.cmd](cfg, args)


if __name__ == "__main__":
    main()
