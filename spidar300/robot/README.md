# SPIDAR300 robot model

`cad/native/` holds the original editable CATIA model. `cad/step/` is the
neutral exchange layer. `docs/` contains assembly references.

When a part changes, update its CAD/STEP source and adjust the primitive visual,
collision and inertial definitions in `../sim/tools/gen_urdf.py`. Then run:

```powershell
python sim/tools/gen_urdf.py
```
