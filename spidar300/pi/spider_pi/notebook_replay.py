"""Execute the original notebook cells, without ROS, IK or the runtime driver.

python -m spider_pi.notebook_replay                   # test_spider_control.ipynb
python -m spider_pi.notebook_replay leg-default       # leg_twister.ipynb default
python -m spider_pi.notebook_replay left-tibia-test   # original left tibia test
python -m spider_pi.notebook_replay --show            # show code, no hardware

No EEPROM calibration cells are included. Notebook port selection and servo
commands are preserved exactly; stop other programs using the servo bus first.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
RECIPES = {
    "spider-test": ("test_spider_control.ipynb", (0, 1, 2, 3)),
    "leg-default": ("leg_twister.ipynb", (0, 1, 5, 6)),
    "left-tibia-test": ("leg_twister.ipynb", (0, 1, 4)),
}


def cells_for(recipe):
    filename, indices = RECIPES[recipe]
    notebook = json.loads((ROOT / filename).read_text())
    return [(index, "".join(notebook["cells"][index]["source"])) for index in indices]


def replay(recipe, namespace=None):
    """Use source straight from the checked-in notebook, not a rewritten driver."""
    scope = {"__name__": "__notebook_replay__"} if namespace is None else namespace
    filename = ROOT / RECIPES[recipe][0]
    try:
        for index, source in cells_for(recipe):
            print(f"Executing {filename.name}, cell {index}", flush=True)
            exec(compile(source, f"{filename}:cell-{index}", "exec"), scope)
    except BaseException:
        # Original blocking waits can raise or be interrupted. Release the
        # commanded joints instead of leaving a failed target energized.
        try:
            if "spider" in scope:
                scope["spider"].disable_torque()
            elif "servo" in scope:
                ids = [113, 123, 133] if recipe == "left-tibia-test" else scope.get("servo_ids", [])
                for servo_id in ids:
                    scope["servo"].write1ByteTxRx(servo_id, 40, 0)
        except Exception as exc:
            print(f"Could not release torque: {exc}", file=sys.stderr)
        raise
    finally:
        if "spider" in scope:
            scope["spider"]._port_handler.closePort()
        elif "port_handler" in scope:
            scope["port_handler"].closePort()
    return scope


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recipe", nargs="?", default="spider-test", choices=RECIPES)
    parser.add_argument("--show", action="store_true", help="print original cells without imports or movement")
    args = parser.parse_args(argv)
    if args.show:
        for index, source in cells_for(args.recipe):
            print(f"# {RECIPES[args.recipe][0]}, cell {index}\n{source}\n")
        return
    sys.path.insert(0, str(ROOT))
    print(f"Recipe: {args.recipe}; original notebook port selection (first serial port).")
    print("Stop spider_pi.robot and other servo programs. Put the belly on the support, feet free.")
    input("Enter = execute original notebook commands; Ctrl-C = cancel: ")
    try:
        replay(args.recipe)
    except KeyboardInterrupt:
        print("Interrupted; torque release requested.")
        raise SystemExit(130)
    except Exception as exc:
        print(f"Original notebook execution failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
    if args.recipe == "spider-test":
        print("Original test finished: default reached, torque disabled by the notebook.")
    else:
        print("Original cell sent its commands; torque remains enabled as in the notebook.")


if __name__ == "__main__":
    main()
