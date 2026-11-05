"""Servo setup and calibration.

  python3 -m spider_pi.calibrate scan                 list servo ids on the bus
  python3 -m spider_pi.calibrate set-id OLD NEW       renumber (only that servo connected)
  python3 -m spider_pi.calibrate zero [JOINT...]      legacy manual EEPROM calibration; not used at startup
  python3 -m spider_pi.calibrate inspect              read servo goals, offsets, limits and status; no motion
  python3 -m spider_pi.calibrate read                 live joint angles from the encoders
  python3 -m spider_pi.calibrate jog JOINT [DELTA]    turn one joint by DELTA rad (sign = direction) and hold
  python3 -m spider_pi.calibrate jog JOINT --to DEG   turn one joint to an angle and hold
  python3 -m spider_pi.calibrate record [JOINT...]    explicitly replace configured zeros with current readings
  python3 -m spider_pi.calibrate default             move to configured default after Enter, hold; no EEPROM writes
  python3 -m spider_pi.calibrate release [JOINT...]   torque off (all 18 by default)

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
    if any(bus.stats.values()):
        print(f"bus hiccups (recovered): {bus.stats}")
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


def cmd_inspect(cfg, args) -> None:
    """Read register snapshots only: no torque, goal or EEPROM writes."""
    servos = ServoMap(cfg["servos"], Hexapod.load(cfg["legs"]).joint_names)
    bus = open_bus(cfg)
    try:
        print("Read-only snapshot; run with robot controller stopped.")
        for c in servos.cal:
            row = bus.diagnostics([c.id])[c.id]
            offset = row["offset"]
            if offset is not None:
                row["offset"] = -(offset & 0x7ff) if offset & 0x800 else offset
            print(f"{c.joint} id={c.id} configured_zero={c.zero} sign={c.sign}: "
                  + " ".join(f"{k}={v}" for k, v in row.items()))
    finally:
        bus.close()


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


JOG_SPEED = 0.5             # rad/s
EXPECTED = {"coxa": "counter-clockwise (seen from above)", "femur": "knee UP", "tibia": "foot OUTWARD"}


def cmd_jog(cfg, args) -> None:
    """Turn one joint by DELTA (or to --to ANGLE) and stay there with torque on; never past legs.yaml limits."""
    hexapod = Hexapod.load(cfg["legs"])
    names = hexapod.joint_names
    if args.joint not in names:
        raise SystemExit(f"unknown joint {args.joint}; one of {names}")
    part = args.joint.rsplit("_", 1)[1]
    lo, hi = hexapod.limits[part]
    servos = ServoMap(cfg["servos"], names)
    i = servos.index(args.joint)
    cal = servos.cal[i]
    bus = open_bus(cfg)
    ticks = bus.positions([cal.id])
    if cal.id not in ticks:
        raise SystemExit(f"servo {cal.id} does not answer")
    start = cal.sign * (ticks[cal.id] - cal.zero) / TICKS_PER_RAD
    wanted = math.radians(args.to) if args.to is not None else start + args.delta
    # never further outside the limits than the joint already is
    target = min(max(wanted, min(lo, start)), max(hi, start))
    if abs(target - wanted) > 1e-6:
        print(f"limit: {args.joint} allowed {math.degrees(lo):+.0f}..{math.degrees(hi):+.0f} deg; "
              f"going to {math.degrees(target):+.1f} instead of {math.degrees(wanted):+.1f}")
    goal = int(round(cal.zero + cal.sign * target * TICKS_PER_RAD))
    bus.torque([cal.id], True)
    bus.move({cal.id: (goal, int(JOG_SPEED * TICKS_PER_RAD))})
    time.sleep(abs(target - start) / JOG_SPEED + 0.3)
    print(f"{args.joint} (id {cal.id}): {math.degrees(start):+.1f} -> {math.degrees(target):+.1f} deg, "
          f"holding (torque on; `release` to let go)")
    if target != start:
        way = "positive" if target > start else "negative"
        print(f"moved {way}: positive should be {EXPECTED[part]}; if it went the other way, "
              f"set sign: {-cal.sign} for {args.joint} in servos.yaml")


def cmd_record(cfg, args) -> None:
    """Take the pose the servos hold right now as the zero pose: present ticks -> servos.yaml. No EEPROM writes."""
    names = Hexapod.load(cfg["legs"]).joint_names
    servos = ServoMap(cfg["servos"], names)
    joints = args.joints or names
    unknown = [j for j in joints if j not in names]
    if unknown:
        raise SystemExit(f"unknown joints {unknown}")
    cal = [servos.cal[servos.index(j)] for j in joints]
    ticks = open_bus(cfg).positions([c.id for c in cal])
    missing = [f"{c.joint} ({c.id})" for c in cal if c.id not in ticks]
    if missing:
        raise SystemExit(f"no answer from {missing}; nothing saved")
    for c in cal:
        print(f"  {c.joint:14s} id {c.id:3d}  zero {ticks[c.id]:4d}  (was {c.zero})")
    servos.save_zeros({c.joint: ticks[c.id] for c in cal})
    print(f"saved to {servos.path}; torque left as it was")


DEFAULT_SPEED = 0.4         # rad/s
DEFAULT_TOLERANCE = 15      # ticks: "arrived" while moving into the zero pose


def cmd_default(cfg, args) -> None:
    """Move to the configured default and hold; never rewrite encoder offsets."""
    hexapod = Hexapod.load(cfg["legs"])
    servos = ServoMap(cfg["servos"], hexapod.joint_names)
    bus = open_bus(cfg)
    try:
        input("Place belly on a high support with feet clear. Enter = move to configured default: ")
        if failed := bus.hold_here(servos.ids):
            raise SystemExit(f"could not hold servos {failed}")
        bus.move({c.id: (c.zero, int(DEFAULT_SPEED * TICKS_PER_RAD)) for c in servos.cal})
        deadline = time.monotonic() + math.pi / DEFAULT_SPEED + 2
        while time.monotonic() < deadline:
            ticks = bus.positions(servos.ids)
            if all(c.id in ticks and abs(ticks[c.id] - c.zero) <= DEFAULT_TOLERANCE for c in servos.cal):
                print("configured default reached; holding. EEPROM and calibration unchanged")
                return
            time.sleep(0.1)
        present = bus.positions(servos.ids)
        bus.move({i: (t, int(DEFAULT_SPEED * TICKS_PER_RAD)) for i, t in present.items()})
        raise SystemExit("default move timed out; holding present positions")
    finally:
        bus.close()


def cmd_release(cfg, args) -> None:
    names = Hexapod.load(cfg["legs"]).joint_names
    servos = ServoMap(cfg["servos"], names)
    joints = args.joints or names
    unknown = [j for j in joints if j not in names]
    if unknown:
        raise SystemExit(f"unknown joints {unknown}")
    open_bus(cfg).torque([servos.cal[servos.index(j)].id for j in joints], False)
    print(f"torque off: {', '.join(joints) if args.joints else 'all 18 servos'}")


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
    sub.add_parser("inspect")
    p = sub.add_parser("jog")
    p.add_argument("joint")
    p.add_argument("delta", type=float, nargs="?", default=0.3, help="rad, sign = direction (default 0.3)")
    p.add_argument("--to", type=float, help="go to this angle in degrees instead (e.g. --to 0)")
    p = sub.add_parser("record")
    p.add_argument("joints", nargs="*", help="only these joints (default: all 18)")
    sub.add_parser("default")
    p = sub.add_parser("release")
    p.add_argument("joints", nargs="*", help="only these joints (default: all 18)")
    args = parser.parse_args(argv)
    cfg = load_config(args.config.resolve())
    {"scan": cmd_scan, "set-id": cmd_set_id, "zero": cmd_zero, "read": cmd_read, "jog": cmd_jog,
     "record": cmd_record, "default": cmd_default, "inspect": cmd_inspect, "release": cmd_release}[args.cmd](cfg, args)


if __name__ == "__main__":
    main()
