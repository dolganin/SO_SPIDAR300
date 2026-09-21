# Robotics engineering

This repository contains engineering sources for SPIDAR300 and SO-ARM100/101.
Robot files follow one navigation rule:

```text
<project>/robot/<component>/<format>/
```

For example, open `spidar300/robot/Pata/Kompas/` to find the editable KOMPAS
leg models, or `arm100/robot/so101/STEP/` to find exchange models for SO-101.

## Robot file map

### SPIDAR300

SPIDAR300 components are stored in `spidar300/robot/`:

| Component | Contents |
| --- | --- |
| `Full_Robot/` | Complete robot assemblies and shared assembly dependencies |
| `Pata/` | Leg assembly and leg parts |
| `cuerpo/` | Body assembly and body parts |
| `Electronica/` | Controller boards and electronic components |
| `Gimbal/` | Camera gimbal |
| `Pinza/` | Gripper |
| `servo/` | Servo models |
| `Tornilleria/` | Fasteners |
| `docs/` | Assembly notes and reference renders |

Useful entry points:

- `spidar300/robot/Full_Robot/Kompas/Antdroid.a3d` — complete KOMPAS assembly.
- `spidar300/robot/Full_Robot/Kompas/Spider_Assembly.a3d` — alternate complete assembly.
- `spidar300/robot/Pata/Kompas/new_Pata.a3d` — leg assembly.
- `spidar300/robot/Pata/STEP/spiderleg1.stp` — leg exchange model.
- `spidar300/robot/cuerpo/Kompas/Cuerpo.a3d` — body assembly.
- `spidar300/sim/` — Isaac Lab simulation code.

### SO-ARM100/101

- `arm100/robot/so100/STEP/` — SO-100 exchange models, including separate
  leader and follower assemblies.
- `arm100/robot/so101/Kompas/` — editable SO-101 KOMPAS sources.
- `arm100/robot/so101/STEP/` — SO-101 exchange models.
- `arm100/robot/so101/Logs/` — CAD application logs.

## Format folders

| Folder | File types |
| --- | --- |
| `Kompas/` | `.a3d`, `.m3d` |
| `STEP/` | `.step`, `.stp` |
| `STL/` | `.stl`; created only when STL files exist |
| `CATIA/` | `.CATPart`, `.CATProduct` |
| `Parasolid/` | `.x_t` |
| `Images/` | Images supplied with a component |
| `Logs/` | CAD application logs |

Keep source code and robot models in Git. Store datasets and trained policies
in Hugging Face Hub.

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://hf.co/cli/install.ps1 | iex"
hf auth login
hf upload 80n3yB4dg3r/SPIDAR300 . --repo-type dataset
```

See `spidar300/HUB.md` for the SPIDAR dataset and model workflow.
