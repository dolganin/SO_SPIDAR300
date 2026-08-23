# SPIDAR300 robot model

`step/` is the editable mechanical source. `docs/` contains assembly
references.

When a part changes, update its STEP source and adjust the primitive visual,
collision and inertial definitions in `../sim/tools/gen_urdf.py`. Then run:

```powershell
python sim/tools/gen_urdf.py
```
