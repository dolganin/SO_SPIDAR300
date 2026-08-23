from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))
sys.path.insert(0, _HERE)

from so_spidar300_lab import kinematics as K
_ROBOT_DIR = Path(_HERE).parents[0] / "so_spidar300_lab" / "assets" / "robot"

MATERIALS = {
    "body": (0.13, 0.13, 0.15, 1.0),
    "coxa": (0.45, 0.45, 0.47, 1.0),
    "femur": (0.42, 0.36, 0.80, 1.0),
    "tibia": (0.10, 0.62, 0.46, 1.0),
    "dark": (0.05, 0.05, 0.06, 1.0),
}

MASS = {"base": 1.10, "coxa": 0.142, "femur": 0.075, "tibia": 0.112, "foot": 0.005}

COXA_W, COXA_H = 0.030, 0.045
FEMUR_W, FEMUR_H = 0.026, 0.020
TIBIA_W, TIBIA_H = 0.024, 0.030
FOOT_R = 0.009

def _box_inertia(m, x, y, z):
    return (m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y))

def _inertial(m, lx, ly, lz, ox=0.0):
    ixx, iyy, izz = _box_inertia(m, lx, ly, lz)
    return (f'    <inertial>\n'
            f'      <origin xyz="{ox:.4f} 0 0"/>\n'
            f'      <mass value="{m:.3f}"/>\n'
            f'      <inertia ixx="{ixx:.3e}" ixy="0" ixz="0" '
            f'iyy="{iyy:.3e}" iyz="0" izz="{izz:.3e}"/>\n'
            f'    </inertial>')

VISUAL_BOXES = {
    "body_lower": (0.196, 0.196, 0.030), "body_upper": (0.180, 0.180, 0.020),
    "camera_mount": (0.040, 0.040, 0.030), "coxa": (K.COXA, COXA_W, COXA_H),
    "femur": (K.FEMUR, FEMUR_W, FEMUR_H), "femur_mirror": (K.FEMUR, FEMUR_W, FEMUR_H),
    "tibia": (K.TIBIA, TIBIA_W, TIBIA_H),
}

def _mesh_visual(key: str, material: str) -> str:
    x, y, z = VISUAL_BOXES[key]
    return (f'    <visual>\n'
            f'      <origin xyz="{x / 2:.4f} 0 0"/>\n'
            f'      <geometry><box size="{x:.4f} {y:.4f} {z:.4f}"/></geometry>\n'
            f'      <material name="{material}"/>\n'
            f'    </visual>')

def _box_collision(length: float, w: float, h: float) -> str:
    return (f'    <collision>\n'
            f'      <origin xyz="{length / 2:.4f} 0 0"/>\n'
            f'      <geometry><box size="{length:.4f} {w:.4f} {h:.4f}"/></geometry>\n'
            f'    </collision>')

def _joint(name, parent, child, xyz, rpy, axis, lo, hi) -> str:
    return (f'  <joint name="{name}" type="revolute">\n'
            f'    <origin xyz="{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}" '
            f'rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>\n'
            f'    <parent link="{parent}"/>\n'
            f'    <child link="{child}"/>\n'
            f'    <axis xyz="{axis}"/>\n'
            f'    <limit lower="{lo:.4f}" upper="{hi:.4f}" '
            f'effort="{K.SERVO_EFFORT}" velocity="{K.SERVO_VELOCITY}"/>\n'
            f'    <dynamics damping="0.05" friction="0.05"/>\n'
            f'  </joint>')

def build() -> str:
    import math
    p: list[str] = ['<?xml version="1.0"?>', '<robot name="so_spidar300">']
    for name, (r, g, b, a) in MATERIALS.items():
        p.append(f'  <material name="{name}"><color rgba="{r} {g} {b} {a}"/></material>')

    bi_xx, bi_yy, bi_zz = _box_inertia(MASS["base"], 0.196, 0.196, 0.05)
    p.append(
        f'  <link name="base_link">\n'
        f'    <inertial><origin xyz="0 0 0"/><mass value="{MASS["base"]:.3f}"/>\n'
        f'      <inertia ixx="{bi_xx:.3e}" ixy="0" ixz="0" iyy="{bi_yy:.3e}" iyz="0" izz="{bi_zz:.3e}"/>\n'
        f'    </inertial>\n'
        f'{_mesh_visual("body_lower", "body")}\n'
        f'{_mesh_visual("body_upper", "body")}\n'
        f'    <collision><origin xyz="0 0 0"/>\n'
        f'      <geometry><box size="0.196 0.196 0.050"/></geometry>\n'
        f'    </collision>\n'
        f'  </link>')

    p.append(
        '  <joint name="camera_joint" type="fixed">\n'
        '    <origin xyz="0.098 0.0 0.045" rpy="0 0 0"/>\n'
        '    <parent link="base_link"/><child link="camera_link"/>\n'
        '  </joint>\n'
        '  <link name="camera_link">\n'
        '    <inertial><mass value="0.02"/>'
        '<inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>\n'
        f'{_mesh_visual("camera_mount", "dark")}\n'
        '  </link>\n'
        '  <joint name="camera_optical_joint" type="fixed">\n'
        '    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>\n'
        '    <parent link="camera_link"/><child link="camera_optical_frame"/>\n'
        '  </joint>\n'
        '  <link name="camera_optical_frame"/>')

    for leg in K.LEGS:
        c, fm, tb = leg.name + "_coxa", leg.name + "_femur", leg.name + "_tibia"
        default = math.radians(leg.default_deg)
        p.append(_joint(K.joint_name(leg.name, "coxa"), "base_link", c,
                        (leg.mount_x, leg.mount_y, 0.0), (0.0, 0.0, default),
                        "0 0 1", *K.COXA_LIMIT))
        p.append(f'  <link name="{c}">\n{_inertial(MASS["coxa"], K.COXA, COXA_W, COXA_H, K.COXA/2)}\n'
                 f'{_mesh_visual("coxa", "coxa")}\n{_box_collision(K.COXA, COXA_W, COXA_H)}\n  </link>')
        p.append(_joint(K.joint_name(leg.name, "femur"), c, fm,
                        (K.COXA, 0.0, K.FEMUR_Z_OFFSET), (0.0, 0.0, 0.0),
                        "0 1 0", *K.FEMUR_LIMIT))
        p.append(f'  <link name="{fm}">\n{_inertial(MASS["femur"], K.FEMUR, FEMUR_W, FEMUR_H, K.FEMUR/2)}\n'
                 f'{_mesh_visual("femur" if leg.side == "left" else "femur_mirror", "femur")}\n'
                 f'{_box_collision(K.FEMUR, FEMUR_W, FEMUR_H)}\n  </link>')
        p.append(_joint(K.joint_name(leg.name, "tibia"), fm, tb,
                        (K.FEMUR, 0.0, 0.0), (0.0, 0.0, 0.0), "0 1 0", *K.TIBIA_LIMIT))
        p.append(f'  <link name="{tb}">\n{_inertial(MASS["tibia"], K.TIBIA, TIBIA_W, TIBIA_H, K.TIBIA/2)}\n'
                 f'{_mesh_visual("tibia", "tibia")}\n{_box_collision(K.TIBIA, TIBIA_W, TIBIA_H)}\n  </link>')
        p.append(
            f'  <joint name="{leg.name}_foot_joint" type="fixed">\n'
            f'    <origin xyz="{K.TIBIA:.4f} 0 0"/>\n'
            f'    <parent link="{tb}"/><child link="{leg.name}_foot"/>\n  </joint>\n'
            f'  <link name="{leg.name}_foot">\n'
            f'    <inertial><mass value="{MASS["foot"]:.3f}"/>'
            f'<inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/></inertial>\n'
            f'    <visual><geometry><sphere radius="{FOOT_R}"/></geometry><material name="dark"/></visual>\n'
            f'    <collision><geometry><sphere radius="{FOOT_R}"/></geometry></collision>\n  </link>')

    p.append("</robot>")
    return "\n".join(p) + "\n"

def write_urdf(output: str | os.PathLike[str] | None = None) -> Path:
    out = Path(output).resolve() if output else _ROBOT_DIR / "so_spidar300.urdf"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="\n") as f:
        f.write(build())
    return out


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate the SO-SPIDAR300 URDF")
    parser.add_argument("--output", default=None, help="Override the manifest runtime URDF path")
    args = parser.parse_args(argv)
    print("written:", write_urdf(args.output))

if __name__ == "__main__":
    main()
