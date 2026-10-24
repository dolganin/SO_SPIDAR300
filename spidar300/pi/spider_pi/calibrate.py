"""Servo setup and calibration.

  python3 -m spider_pi.calibrate scan                 list servo ids on the bus
  python3 -m spider_pi.calibrate set-id OLD NEW       renumber (only that servo connected)
  python3 -m spider_pi.calibrate zero [JOINT...]      torque off, pose by hand, write EEPROM offsets
  python3 -m spider_pi.calibrate read                 live joint angles from the encoders
  python3 -m spider_pi.calibrate jog JOINT [DELTA]    move one joint by DELTA rad to check its sign

Zero pose (legs.yaml): every coxa points straight out from the body, every femur is
horizontal, every tibia hangs vertically. Hold the robot in the air.
Positive: coxa counter-clockwise seen from above, femur knee up, tibia foot outward.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import yaml

from spider_gait.kinematics import Hexapod

from .robot import _HERE, load_config
from .servos import TICKS_PER_RAD, ServoMap
from .sts import StsBus


def open_bus(cfg) -> StsBus:
    return StsBus(cfg["bus"]["port"], int(cfg["bus"]["baudrate"]))


def cmd_scan(cfg, args) -> None:
    bus = open_bus(cfg)
    found = [i for i in range(args.first, args.last + 1) if bus.ping(i)]
    print(f"servos on {cfg['bus']['port']}: {found or 'none'} ({len(found)})")
    expected = {c["id"] for c in yaml.safe_load(open(cfg["servos"]))["servos"].values()}
    if not found:
        print("nothing answers: check servo power, the port (ls /dev/ttyACM*) and that no other program uses it")
    elif missing := sorted(expected - set(found)):
        print(f"in servos.yaml but not answering: {missing}")
    if extra := sorted(set(found) - expected):
        print(f"answering but not in servos.yaml: {extra}")


def cmd_set_id(cfg, args) -> None:
    bus = open_bus(cfg)
    if not bus.ping(args.old):
        raise SystemExit(f"no servo with id {args.old}")
    bus.set_id(args.old, args.new)
    print(f"id {args.old} -> {args.new}: {'ok' if bus.ping(args.new) else 'FAILED'}")


def cmd_zero(cfg, args) -> None:
    """Like leg_twister.ipynb: write each servo's offset so the current pose reads 2048."""
    names = Hexapod.load(cfg["legs"]).joint_names
    servos = ServoMap(cfg["servos"], names)
    joints = args.joints or names
    unknown = [j for j in joints if j not in names]
    if unknown:
        raise SystemExit(f"unknown joints {unknown}")
    cal = [servos.cal[names.index(j)] for j in joints]
    bus = open_bus(cfg)
    bus.torque([c.id for c in cal], False)
    print(f"Torque is off for {len(cal)} servos. Put them in the zero pose (see --help), hold, press Enter.")
    input()
    for c in cal:
        offset = bus.center_here(c.id)
        print(f"  {c.joint:14s} id {c.id:3d}  offset {offset:+5d}")
    servos.save_zeros({c.joint: 2048 for c in cal})
    print(f"offsets written to the servos; zero = 2048 in {servos.path}")


LEG_TITLES = {"lf": "left front", "lm": "left middle", "lr": "left rear",
              "rf": "right front", "rm": "right middle", "rr": "right rear"}


def read_table(hexapod: Hexapod, servos: ServoMap, ticks: dict[int, int]) -> list[str]:
    """One line per leg: id, raw ticks and joint angle for coxa / femur / tibia."""
    lines = [f"{'':16s}" + "".join(f"  {j:<22s}" for j in hexapod.joints),
             f"{'':16s}" + f"{'  id  ticks    angle':<24s}" * len(hexapod.joints)]
    for leg in hexapod.legs:
        cells = []
        for j in hexapod.joints:
            c = servos.cal[servos.index(f"leg_{leg}_{j}")]
            if c.id in ticks:
                angle = c.sign * (ticks[c.id] - c.zero) / TICKS_PER_RAD
                cells.append(f"{c.id:3d}  {ticks[c.id]:5d}  {math.degrees(angle):+7.1f}°")
            else:
                cells.append(f"{c.id:3d}    ---   no reply")
        lines.append(f"{leg} {LEG_TITLES[leg]:13s}" + "".join(f"{x:<24s}" for x in cells))
    return lines


def cmd_read(cfg, args) -> None:
    hexapod = Hexapod.load(cfg["legs"])
    servos = ServoMap(cfg["servos"], hexapod.joint_names)
    bus = open_bus(cfg)
    print("angles in the legs.yaml convention: 0 = zero pose; Ctrl-C to stop")
    drawn = 0
    try:
        while True:
            ticks = bus.positions(servos.ids)
            lines = read_table(hexapod, servos, ticks)
            if not ticks:
                lines.append("no servo answers: check servo power and the port (ls /dev/ttyACM*)")
            else:
                lines.append("")
            if drawn:
                sys.stdout.write(f"\x1b[{drawn}F")
            sys.stdout.write("".join(f"\x1b[2K{l}\n" for l in lines))
            sys.stdout.flush()
            drawn = len(lines)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


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
    p.add_argument("--last", type=int, default=253)
    p = sub.add_parser("set-id")
    p.add_argument("old", type=int)
    p.add_argument("new", type=int)
    p = sub.add_parser("zero")
    p.add_argument("joints", nargs="*", help="only these joints (default: all 18)")
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
