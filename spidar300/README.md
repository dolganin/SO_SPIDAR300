# SPIDAR300

SPIDAR300 is an 18-DoF hexapod with editable mechanical sources, URDF/USD
simulation assets, Isaac Lab tasks, teleoperation and LeRobot training tools.

```powershell
python sim/tools/gen_urdf.py
python sim/scripts/convert_urdf_to_usd.py --headless
```

- `robot/` contains the CAD/STEP model and assembly references.
- `sim/` contains the Python package, tools, scripts and simulation assets.
- `HUB.md` defines where datasets and trained models are published.
