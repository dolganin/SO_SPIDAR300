# SPIDAR300 robot model

`cad/native/` is the editable mechanical source. `step/` is reserved for
neutral exports. `docs/` contains assembly references.

When a part changes, update its native CAD source, export STEP when needed, and
adjust the primitive visual,
collision and inertial definitions in `../sim/tools/gen_urdf.py`. Then run:

```powershell
python sim/tools/gen_urdf.py
```
