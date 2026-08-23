# SPIDAR300 simulation

This directory contains the Isaac Lab extension, URDF/USD generation tools,
teleoperation, recording, replay and LeRobot training scripts.

```powershell
python -m pip install -e source/so_spidar300_lab
python tools/gen_urdf.py
python scripts/convert_urdf_to_usd.py --headless
```

Use `../HUB.md` for dataset and model publication.
