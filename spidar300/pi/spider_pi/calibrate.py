"""Utilities using the original spider_control.Spider; no EEPROM writes.

python3 -m spider_pi.calibrate default   # Enter, original default, hold
python3 -m spider_pi.calibrate read      # read positions
python3 -m spider_pi.calibrate inspect   # read register snapshot
python3 -m spider_pi.calibrate release  # disable torque
python3 -m spider_pi.calibrate scan     # check configured IDs
"""
import argparse
import math
import sys
import time
from pathlib import Path

from spider_gait.kinematics import Hexapod
from .robot import _HERE, load_config
from .servos import ServoMap, TICKS_PER_RAD
from .spider_bus import SpiderBus

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



def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=_HERE / "config.yaml")
    parser.add_argument("cmd", choices=["default", "read", "inspect", "release", "scan"])
    args = parser.parse_args(argv)
    cfg = load_config(args.config.resolve())
    bus = SpiderBus(cfg["bus"]["port"])
    try:
        robot = Hexapod.load(cfg["legs"])
        servos = ServoMap(cfg["servos"], robot.joint_names)
        for c in servos.cal:
            c.zero = bus.default_positions[c.id]
        if args.cmd == "default":
            input("Place belly on the support, feet clear. Enter = original Spider default: ")
            bus.default(blocking=True)
            print("Spider default reached; torque stays on; no further movement")
        elif args.cmd == "release":
            bus.torque(servos.ids, False)
        elif args.cmd == "scan":
            print({i: bus.ping(i) for i in servos.ids})
        elif args.cmd == "inspect":
            for c in servos.cal:
                print(f"{c.joint} id={c.id}: {bus.diagnostics([c.id])[c.id]}")
        else:
            while True:
                print("\n".join(read_table(robot, servos, bus.positions(servos.ids))), flush=True)
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()


if __name__ == "__main__":
    main()
