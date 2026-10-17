"""One-off conversion of the CAD-exported sim/urdf/robot.urdf into spider_description.

Writes config/legs.yaml (leg table, idealised leg geometry, joint limits),
config/links.yaml (mesh offsets, inertials, collision boxes in the new link frames)
and copies the visual meshes under the new names. After this, legs.yaml is the
hand-edited source of truth; rerun only when the CAD changes.

New conventions (all legs share one kinematic chain, mounted at a yaw):
  base_link  x forward, z up
  coxa       axis +z,  0 = leg along its mount yaw,  + = counter-clockwise from above
  femur      axis -y,  0 = femur horizontal,         + = knee up
  tibia      axis -y,  0 = tibia vertical (femur 0), + = foot swings outward/up
"""
from __future__ import annotations

import argparse
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pybullet as p
import trimesh

_SIM = Path(__file__).resolve().parents[1]
_OLD_URDF = _SIM / "urdf" / "robot.urdf"
_PKG = _SIM / "ros2_ws" / "src" / "spider_description"

# Body direction (CAD base_link frame) that becomes +x of the new base_link, degrees.
FORWARD_IN_CAD = -45.0
# CAD leg index for each new leg name, in /joint_command order.
LEG_MAP = {"lf": 3, "lm": 2, "lr": 1, "rf": 6, "rm": 5, "rr": 4}
REFERENCE_LEG = 2
FOOT_IN_TIBIA = np.array([0.1111, 0.1255, -0.0193])
FOOT_RADIUS = 0.008
TIBIA_BOX_CLEARANCE = 0.04
LIMIT_MARGIN = 0.05
GEOMETRY_TOLERANCE = 1e-4
SWEEP_STEP = 0.01
PARTS = ("coxa", "femur", "tibia")


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def trans(v):
    t = np.eye(4)
    t[:3, 3] = v
    return t


def rpy_of(r):
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    return [math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])]


def origin_of(j):
    o = j.find("origin")
    xyz = [float(v) for v in o.get("xyz").split()]
    r, pi, y = (float(v) for v in o.get("rpy").split())
    return trans(xyz) @ rot_z(y) @ rot_y(pi) @ rot_x(r)


class OldLeg:
    """CAD chain of one leg, expressed in the new base_link frame."""

    def __init__(self, joints, n, base):
        self.n = n
        self.base = base
        self.origins = [origin_of(joints[f"Leg_{n}_{part}_joint"]) for part in PARTS]
        self.limits = [tuple(float(joints[f"Leg_{n}_{part}_joint"].find("limit").get(k)) for k in ("lower", "upper"))
                       for part in PARTS]

    def frames(self, q):
        t, out = self.base, []
        for origin, a in zip(self.origins, q):
            t = t @ origin @ rot_z(a)
            out.append(t)
        return out

    def foot(self, q):
        return (self.frames(q)[2] @ np.append(FOOT_IN_TIBIA, 1.0))[:3]


def solve_zero(leg: OldLeg):
    """Old femur/tibia angles at which the femur is horizontal and the tibia vertical."""
    def bisect(f, lo, hi):
        flo = f(lo)
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if (f(mid) > 0) == (flo > 0):
                lo, flo = mid, f(mid)
            else:
                hi = mid
        return 0.5 * (lo + hi)

    qf = bisect(lambda a: leg.frames([0, a, 0])[2][2, 3] - leg.frames([0, a, 0])[1][2, 3], -1.0, 1.0)
    yaw = leg_yaw(leg)
    def foot_dx(a):
        fr = leg.frames([0, qf, a])
        d = rot_z(-yaw)[:3, :3] @ (leg.foot([0, qf, a]) - fr[2][:3, 3])
        return d[0]
    qt = bisect(foot_dx, -1.2, 1.2)
    return qf, qt


def leg_yaw(leg: OldLeg) -> float:
    ax = leg.frames([0, 0, 0])[1][:3, 2]
    return math.atan2(-ax[0], ax[1])


def canonical_frames(mount, yaw, geo, q):
    tc = trans(mount) @ rot_z(yaw) @ rot_z(q[0])
    tf = tc @ trans(geo["coxa"]) @ rot_y(-q[1])
    tt = tf @ trans(geo["femur"]) @ rot_y(-q[2])
    return [tc, tf, tt]


def radial_yaw(leg: OldLeg) -> float:
    o = leg.frames([0, 0, 0])[0][:3, 3]
    return math.atan2(o[1], o[0])


def leg_geometry(leg: OldLeg, qc0, qf0, qt0):
    yaw = radial_yaw(leg)
    tc, tf, tt = leg.frames([qc0, qf0, qt0])
    r = rot_z(-yaw)[:3, :3]
    o = tc[:3, 3]
    femur_pt, tibia_pt, foot = (r @ (x - o) for x in (tf[:3, 3], tt[:3, 3], leg.foot([qc0, qf0, qt0])))
    return yaw, o, {
        "coxa": femur_pt.tolist(),
        "femur": (tibia_pt - femur_pt).tolist(),
        "foot": (foot - tibia_pt).tolist(),
    }


def collision_limits(old_urdf, legs, zero):
    """Per-joint collision-free interval around the new zero, other joints held at zero."""
    p.connect(p.DIRECT)
    robot = p.loadURDF(str(old_urdf), useFixedBase=True)
    index = {p.getJointInfo(robot, i)[12].decode(): i for i in range(p.getNumJoints(robot))}
    limits = {}
    for name, leg in legs.items():
        links = [index[f"Leg_{leg.n}_{part}"] for part in PARTS]
        neighbours = [index[f"Leg_{o.n}_{part}"] for o in legs.values() if o is not leg for part in PARTS]

        def pose(q):
            for lg in legs.values():
                for part, a in zip(PARTS, zero[lg.n]):
                    p.resetJointState(robot, index[f"Leg_{lg.n}_{part}"], a)
            for part, a in zip(PARTS, q):
                p.resetJointState(robot, index[f"Leg_{leg.n}_{part}"], a)

        def pairs(k):
            moving = links[k:]
            others = [-1] + links[:max(0, k - 1)] + neighbours
            return [(a, b) for a in moving for b in others if not (k == 0 and b == -1 and a == links[0])]

        def hits(q, k):
            pose(q)
            p.performCollisionDetection()
            return {(a, b) for a, b in pairs(k)
                    if p.getClosestPoints(robot, robot, 0.0, linkIndexA=a, linkIndexB=b)}

        limits[name] = []
        for k in range(3):
            q0 = list(zero[leg.n])
            at_zero = hits(q0, k)
            lo_old, hi_old = leg.limits[k]
            ends = []
            for direction in (-1, 1):
                a = q0[k]
                while lo_old <= a + direction * SWEEP_STEP <= hi_old:
                    q = list(q0)
                    q[k] = a + direction * SWEEP_STEP
                    if hits(q, k) - at_zero:
                        break
                    a = q[k]
                ends.append(a)
            limits[name].append(ends)
    p.disconnect()
    return limits


def fmt(v, nd=6):
    return "[" + ", ".join(f"{x:.{nd}g}" if abs(x) > 1e-12 else "0" for x in v) + "]"


def inertia_str(i):
    return (f"{{ixx: {i[0, 0]:.4e}, ixy: {i[0, 1]:.4e}, ixz: {i[0, 2]:.4e}, "
            f"iyy: {i[1, 1]:.4e}, iyz: {i[1, 2]:.4e}, izz: {i[2, 2]:.4e}}}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--urdf", type=Path, default=_OLD_URDF)
    parser.add_argument("--out", type=Path, default=_PKG)
    args = parser.parse_args()

    root = ET.parse(args.urdf).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    links = {l.get("name"): l for l in root.findall("link")}
    base = rot_z(-math.radians(FORWARD_IN_CAD))
    legs = {name: OldLeg(joints, n, base) for name, n in LEG_MAP.items()}

    zero = {}
    geos = {}
    for name, leg in legs.items():
        qf0, qt0 = solve_zero(leg)
        qc0 = math.remainder(radial_yaw(leg) - leg_yaw(leg), math.tau)
        zero[leg.n] = [qc0, qf0, qt0]
        geos[name] = leg_geometry(leg, qc0, qf0, qt0)
    ref_name = next(k for k, v in LEG_MAP.items() if v == REFERENCE_LEG)
    reference = legs[ref_name]
    ref = geos[ref_name][2]
    geo = {k: list(v) for k, v in ref.items()}
    geo_dev = {}
    for name, (_, _, g) in geos.items():
        dev = geo_dev[name] = max(np.abs(np.subtract(g[k], ref[k])).max() for k in ref)
        print(f"leg {name} (CAD {LEG_MAP[name]}): yaw {math.degrees(geos[name][0]):8.3f} deg, "
              f"geometry deviation from reference {dev * 1000:.2f} mm")

    sweep = collision_limits(args.urdf, legs, zero)

    # new q = old q - old zero (coxa) | old zero - old q (femur, tibia); both maps are involutions
    def to_new(k, a, z):
        return a - z if k == 0 else z - a

    def to_old(k, a, z):
        return a + z if k == 0 else z - a

    out_meshes = args.out / "meshes" / "visual"
    out_meshes.mkdir(parents=True, exist_ok=True)

    legs_yaml = [
        "# Single source of truth for the hexapod: legs, joints, geometry, limits.",
        "# Generated once by sim/tools/build_description.py from the CAD export, then edited by hand.",
        "# Joint names are leg_<leg>_<joint>; order here is the order of /joint_command.",
        "#",
        "# Frames: base_link x forward, z up. Every leg has the same chain, rotated by `yaw`:",
        "#   coxa  axis +z  0 = leg along yaw          + = counter-clockwise from above",
        "#   femur axis -y  0 = femur horizontal       + = knee up",
        "#   tibia axis -y  0 = tibia vertical down    + = foot swings outward/up",
        "",
        "joints: [coxa, femur, tibia]",
        "",
        "legs:",
    ]
    links_yaml = [
        "# Generated by sim/tools/build_description.py: mesh placement, inertials and",
        "# collision boxes expressed in the link frames defined by legs.yaml. Do not edit.",
        "",
    ]
    for name, leg in legs.items():
        yaw, mount, _ = geos[name]
        legs_yaml.append(f"  - {{name: {name}, xyz: {fmt(mount)}, yaw: {yaw:.6f}}}")

    limits = {}
    for k, part in enumerate(PARTS):
        lo, hi = -math.inf, math.inf
        for name, leg in legs.items():
            z = zero[leg.n][k]
            a, b = sorted(to_new(k, v, z) for v in sweep[name][k])
            lo, hi = max(lo, a + LIMIT_MARGIN), min(hi, b - LIMIT_MARGIN)
        limits[part] = (lo, hi)
        print(f"{part:6s} limits [{lo:+.3f}, {hi:+.3f}] rad  = [{math.degrees(lo):+.1f}, {math.degrees(hi):+.1f}] deg")

    effort = float(joints["Leg_1_coxa_joint"].find("limit").get("effort"))
    velocity = float(joints["Leg_1_coxa_joint"].find("limit").get("velocity"))
    legs_yaml += [
        "",
        "# Leg chain in the coxa frame: femur axis, tibia axis (in femur frame), foot (in tibia frame).",
        "geometry:",
        f"  coxa:  {fmt(geo['coxa'])}",
        f"  femur: {fmt(geo['femur'])}",
        f"  foot:  {fmt(geo['foot'])}",
        f"  foot_radius: {FOOT_RADIUS}",
        "",
        "# Collision-free range of each joint with the other joints at zero, minus a margin.",
        "limits:",
    ]
    legs_yaml += [f"  {part}: [{lo:.3f}, {hi:.3f}]" for part, (lo, hi) in limits.items()]
    legs_yaml += [
        "",
        "# Pose the robot starts in (lying on its belly, legs spread out) and returns to on lie_down.",
        "rest_pose: {coxa: 0.0, femur: 0.0, tibia: 1.5}",
        "",
        "servo:",
        f"  effort: {effort}       # N*m",
        f"  velocity: {velocity}     # rad/s",
        "  damping: 0.05",
        "  friction: 0.02",
        "",
        "# gz-sim JointPositionController. Velocity mode: the PID output is a joint velocity,",
        "# clamped to servo.velocity; the physics engine tracks it with at most servo.effort,",
        "# like a position servo. (Force-mode PID with explicit D is unstable on the 44 g femur.)",
        "controller:",
        "  use_velocity_commands: true",
        "  p: 12.0       # 1/s: velocity per rad of error",
        "  i: 0.0",
        "  d: 0.0",
        "",
        "foot_friction: 1.0",
    ]

    def link_entry(key, old_link, frame_new, frame_old, mesh_src, box_fn):
        off = np.linalg.inv(frame_new) @ frame_old
        el = links[old_link]
        inert = el.find("inertial")
        com = np.array([float(v) for v in inert.find("origin").get("xyz").split()])
        mass = float(inert.find("mass").get("value"))
        ia = {k: float(v) for k, v in inert.find("inertia").attrib.items()}
        i_old = np.array([[ia["ixx"], ia["ixy"], ia["ixz"]], [ia["ixy"], ia["iyy"], ia["iyz"]],
                          [ia["ixz"], ia["iyz"], ia["izz"]]])
        r = off[:3, :3]
        i_new = r @ i_old @ r.T
        com_new = (off @ np.append(com, 1.0))[:3]
        mesh = trimesh.load(mesh_src, force="mesh")
        verts = (off[:3, :3] @ mesh.vertices.T).T + off[:3, 3]
        center, size = box_fn(verts)
        shutil.copyfile(mesh_src, out_meshes / f"{key}.stl")
        links_yaml.extend([
            f"{key}:",
            f"  mesh: {key}.stl",
            f"  visual: {{xyz: {fmt(off[:3, 3])}, rpy: {fmt(rpy_of(off[:3, :3]))}}}",
            f"  mass: {mass}",
            f"  com: {fmt(com_new)}",
            f"  inertia: {inertia_str(i_new)}",
            f"  box: {{xyz: {fmt(center, 4)}, size: {fmt(size, 4)}}}",
        ])

    def aabb(v):
        lo, hi = np.percentile(v, 1, axis=0), np.percentile(v, 99, axis=0)
        return (lo + hi) / 2, hi - lo

    def tibia_box(v):
        lo, hi = np.percentile(v, 1, axis=0), np.percentile(v, 99, axis=0)
        # keep the box clear of the foot sphere even with the tibia tilted
        lo[2] = geo["foot"][2] + TIBIA_BOX_CLEARANCE
        return (lo + hi) / 2, hi - lo

    def base_box(v):
        z = v[:, 2]
        lo, hi = np.percentile(z, 1), np.percentile(z, 99)
        r = np.percentile(np.hypot(v[:, 0], v[:, 1]), 90)
        return np.array([0.0, 0.0, (lo + hi) / 2]), np.array([2 * r * 0.75, 2 * r * 0.75, hi - lo])

    link_entry("base_link", "base_link", np.eye(4), base, args.urdf.parent / "meshes" / "base_link.stl", base_box)
    for name, leg in legs.items():
        yaw, mount, _ = geos[name]
        new = canonical_frames(mount, yaw, geo, [0, 0, 0])
        # Femur and tibia are the same printed parts on every leg; a leg whose CAD chain
        # deviates from the reference borrows the reference parts so mesh and joints agree.
        src = legs[name] if geo_dev[name] < GEOMETRY_TOLERANCE else reference
        src_new = canonical_frames(geos[ref_name][1], geos[ref_name][0], geo, [0, 0, 0])
        for k, part in enumerate(PARTS):
            fn = tibia_box if part == "tibia" else aabb
            if k == 0 or src is leg:
                frame_new, frame_old, n = new[k], leg.frames(zero[leg.n])[k], leg.n
            else:
                frame_new, frame_old, n = src_new[k], src.frames(zero[src.n])[k], src.n
            link_entry(f"leg_{name}_{part}", f"Leg_{n}_{part}", frame_new, frame_old,
                       args.urdf.parent / "meshes" / f"Leg_{n}_{part}.stl", fn)

    # Round-trip check: the idealised chain reproduces the CAD foot over random poses.
    rng = np.random.default_rng(0)
    for name, leg in legs.items():
        yaw, mount, _ = geos[name]
        err = 0.0
        for _ in range(200):
            qn = rng.uniform([l for l, _ in limits.values()], [h for _, h in limits.values()])
            qo = [to_old(k, a, z) for k, (a, z) in enumerate(zip(qn, zero[leg.n]))]
            fn = (canonical_frames(mount, yaw, geo, qn)[2] @ np.append(geo["foot"], 1.0))[:3]
            err = max(err, np.linalg.norm(fn - leg.foot(qo)))
        print(f"leg {name}: max foot error vs CAD chain {err * 1000:.2f} mm")

    (args.out / "config").mkdir(parents=True, exist_ok=True)
    (args.out / "config" / "legs.yaml").write_text("\n".join(legs_yaml) + "\n")
    (args.out / "config" / "links.yaml").write_text("\n".join(links_yaml) + "\n")
    print("written:", args.out / "config")


if __name__ == "__main__":
    main()
