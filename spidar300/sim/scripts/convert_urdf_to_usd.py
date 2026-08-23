from __future__ import annotations

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="URDF -> USD for SO-SPIDAR300")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROBOT_DIR = os.path.abspath(os.path.join(
    _HERE, "..", "so_spidar300_lab", "assets", "robot"))
URDF = os.path.join(_ROBOT_DIR, "so_spidar300.urdf")

def _structure_and_color(usd_path: str) -> None:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    palette = {
        "body": (0.13, 0.13, 0.15),
        "coxa": (0.45, 0.45, 0.47),
        "femur": (0.42, 0.36, 0.80),
        "tibia": (0.10, 0.62, 0.46),
        "servo": (0.04, 0.04, 0.05),
    }
    stage = Usd.Stage.Open(usd_path)
    mats = {}
    for name, rgb in palette.items():
        mpath = f"/Looks/{name}"
        mat = UsdShade.Material.Define(stage, mpath)
        sh = UsdShade.Shader.Define(stage, mpath + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.1)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        mats[name] = mat

    def part_of(link_name: str) -> str | None:
        if link_name == "base_link":
            return "body"
        for kind in ("coxa", "femur", "tibia"):
            if link_name.endswith("_" + kind):
                return kind
        return None

    servo_size = Gf.Vec3f(0.020, 0.041, 0.038)
    n_bound, n_servos = 0, 0
    for prim in stage.Traverse():
        part = part_of(prim.GetName())
        if part is None or not prim.IsA(UsdGeom.Xform):
            continue
        vis = prim.GetChild("visuals")
        if vis and vis.IsValid():
            UsdShade.MaterialBindingAPI.Apply(vis).Bind(
                mats[part], bindingStrength=UsdShade.Tokens.strongerThanDescendants
            )
            n_bound += 1
        if part in ("coxa", "femur", "tibia"):
            cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendChild("servo_vis"))
            cube.GetSizeAttr().Set(1.0)
            UsdGeom.Xformable(cube.GetPrim()).AddScaleOp().Set(servo_size)
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mats["servo"])
            n_servos += 1
    n_grip = _apply_grip_material(stage)
    stage.Save()
    print(f"[OK] structured visuals: {n_bound} part materials bound, {n_servos} servo blocks added")
    print(f"[OK] grip physics material bound to {n_grip} collision scopes")

def _apply_grip_material(stage) -> int:
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    mat = UsdShade.Material.Define(stage, "/Looks/grip_phys")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr().Set(1.3)
    pm.CreateDynamicFrictionAttr().Set(1.1)
    pm.CreateRestitutionAttr().Set(0.0)
    px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
    px.CreateFrictionCombineModeAttr().Set("max")

    n = 0
    for prim in stage.Traverse():
        if prim.GetName() == "collisions":
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                mat,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose="physics",
            )
            n += 1
    return n

def main() -> None:
    cfg = UrdfConverterCfg(
        asset_path=URDF,
        usd_dir=_ROBOT_DIR,
        usd_file_name="so_spidar300.usd",
        fix_base=False,
        merge_fixed_joints=True,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=8.0, damping=0.3),
        ),
    )
    converter = UrdfConverter(cfg)
    print("[OK] USD written to:", converter.usd_path)
    _structure_and_color(converter.usd_path)

if __name__ == "__main__":
    main()
    simulation_app.close()
