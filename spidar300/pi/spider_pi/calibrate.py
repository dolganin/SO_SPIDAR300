"""Utilities using the original spider_control.Spider; no EEPROM writes.

python3 -m spider_pi.calibrate default   # Enter, original default, hold
python3 -m spider_pi.calibrate read      # read positions
python3 -m spider_pi.calibrate jog leg_lf_tibia 0.1  # small relative move, radians
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



def jog(bus, robot, servos, joint, delta):
    """One small relative movement through the existing Spider methods."""
    if joint not in robot.joint_names:
        raise ValueError(f"unknown joint: {joint}")
    if not math.isfinite(delta) or not 0 < abs(delta) <= 0.15:
        raise ValueError("delta must be nonzero and at most 0.15 rad (8.6 degrees)")
    c = servos.cal[servos.index(joint)]
    input(f"{joint} id={c.id}: move {math.degrees(delta):+.1f} degrees. Feet clear; Enter = move: ")
    positions = bus.positions([c.id])
    if c.id not in positions:
        raise IOError(f"cannot read servo {c.id}: {getattr(bus, 'position_errors', {})}")
    start = positions[c.id]
    goal = start + round(c.sign * delta * TICKS_PER_RAD)
    lo, hi = robot.limits[joint.rsplit("_", 1)[1]]
    lo, hi = max(lo, c.min_angle), min(hi, c.max_angle)
    angle = c.sign * (goal - c.zero) / TICKS_PER_RAD
    if not 0 <= goal <= 4095 or not lo <= angle <= hi:
        raise ValueError("requested move is outside the encoder or model limits")
    print(f"{joint}: {start} -> {goal} ticks", flush=True)
    try:
        # Install the measured goal before enabling torque to avoid a stale target.
        bus.move({c.id: (start, 100)})
        bus.torque([c.id], True)
        bus.move({c.id: (goal, 100)})
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            measured = bus.positions([c.id]).get(c.id)
            if measured is None:
                raise IOError(f"cannot read servo {c.id}: {getattr(bus, 'position_errors', {})}")
            if abs(measured - goal) <= 5:
                print(f"Reached {measured}; holding. No calibration changes.")
                return
            time.sleep(0.05)
        raise TimeoutError(f"servo {c.id} did not reach {goal}; last position {measured}")
    except BaseException:
        bus.torque([c.id], False)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=_HERE / "config.yaml")
    parser.add_argument("cmd", choices=["default", "read", "inspect", "release", "scan", "jog"])
    parser.add_argument("joint", nargs="?", help="joint for jog, e.g. leg_lf_tibia")
    parser.add_argument("delta", nargs="?", type=float, default=0.1, help="relative jog angle in radians")
    args = parser.parse_args(argv)
    if args.cmd == "jog" and args.joint is None:
        parser.error("jog requires a joint name")
    if args.cmd != "jog" and args.joint is not None:
        parser.error("joint and delta are only supported for jog")
    cfg = load_config(args.config.resolve())
    bus = SpiderBus(cfg["bus"]["port"])
    try:
        robot = Hexapod.load(cfg["legs"])
        servos = ServoMap(cfg["servos"], robot.joint_names)
        bus.configure_defaults({c.id: c.zero for c in servos.cal})
        if args.cmd == "jog":
            jog(bus, robot, servos, args.joint, args.delta)
        elif args.cmd == "default":
            input("Place belly on the support, feet clear. Enter = measured reference via Spider: ")
            bus.default(blocking=True)
            print("Measured reference reached via Spider; torque stays on; no further movement")
        elif args.cmd == "release":
            bus.torque(servos.ids, False)
        elif args.cmd == "scan":
            print({i: bus.ping(i) for i in servos.ids})
        elif args.cmd == "inspect":
            for c in servos.cal:
                print(f"{c.joint} id={c.id}: {bus.diagnostics([c.id])[c.id]}")
        else:
            terminal = sys.stdout.isatty()
            if terminal:
                sys.stdout.write("\x1b[2J\x1b[H\x1b[?25l")
                sys.stdout.flush()
            try:
                while True:
                    table = "\n".join(read_table(robot, servos, bus.positions(servos.ids)))
                    if terminal:
                        sys.stdout.write("\x1b[H" + table + "\nCtrl+C — stop reading\x1b[J")
                        sys.stdout.flush()
                    else:
                        print(table, flush=True)
                    time.sleep(0.5)
            finally:
                if terminal:
                    sys.stdout.write("\x1b[?25h\n")
                    sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()


if __name__ == "__main__":
    main()
