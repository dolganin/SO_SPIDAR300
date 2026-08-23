from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))
from so_spidar300_lab import kinematics as K

URDF = os.path.abspath(os.path.join(
    _HERE, "..", "so_spidar300_lab",
    "assets", "robot", "so_spidar300.urdf"))

def _rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])

def _axis_rot(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s, C = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])

def _T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M

class Urdf:
    def __init__(self, path: str):
        self.root = ET.parse(path).getroot()
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.parent_joint = {}
        for j in self.joints.values():
            self.parent_joint[j.find("child").get("link")] = j

    def link_pose(self, link: str, q: dict) -> np.ndarray:
        T = np.eye(4)
        chain = []
        while link in self.parent_joint:
            j = self.parent_joint[link]
            chain.append(j)
            link = j.find("parent").get("link")
        for j in reversed(chain):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())]
            T = T @ _T(_rpy(*rpy), np.array(xyz))
            if j.get("type") == "revolute":
                axis = [float(v) for v in j.find("axis").get("xyz").split()]
                T = T @ _T(_axis_rot(axis, q.get(j.get("name"), 0.0)), np.zeros(3))
        return T

def _q_from_legvec(c, f, t) -> dict:
    return {K.joint_name(l.name, jn): v
            for l in K.LEGS for jn, v in zip(K.JOINT_ORDER, (c, f, t))}

def main() -> int:
    u = Urdf(URDF)
    ok = True

    rev = [n for n, j in u.joints.items() if j.get("type") == "revolute"]
    if sorted(rev) != sorted(K.JOINT_NAMES_18):
        print("FAIL structure: revolute joints != expected 18"); ok = False
    else:
        print(f"OK   structure: 18 revolute joints, {len(u.links)} links")

    urdf_dir = os.path.dirname(URDF)
    for mesh in {m.get("filename") for m in u.root.iter("mesh")}:
        if not os.path.exists(os.path.join(urdf_dir, mesh)):
            print(f"FAIL mesh missing: {mesh}"); ok = False

    for name, j in u.joints.items():
        if j.get("type") != "revolute":
            continue
        joint_kind = name.split("_")[1]
        lo, hi = float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))
        klo, khi = K.JOINT_LIMITS[joint_kind]
        if abs(lo - klo) > 1e-3 or abs(hi - khi) > 1e-3:
            print(f"FAIL limit {name}: ({lo:.3f},{hi:.3f}) != ({klo:.3f},{khi:.3f})"); ok = False
    print("OK   limits match kinematics")

    c, f, t = K.default_stance_angles()
    q = _q_from_legvec(c, f, t)
    max_err = 0.0
    for leg in K.LEGS:
        urdf_foot = u.link_pose(f"{leg.name}_foot", q)[:3, 3]
        model_foot = K.fk_leg_body(leg, c, f, t)
        max_err = max(max_err, np.linalg.norm(urdf_foot - model_foot))
    print(f"{'OK  ' if max_err < 5e-5 else 'FAIL'} stance FK vs model: max err {max_err:.2e} m")
    ok &= max_err < 5e-5
    foot0 = u.link_pose("lf_foot", q)[:3, 3]
    print(f"     lf foot @stance (mm): {np.round(foot0 * 1000, 1)}")

    import random
    random.seed(1)
    max_err = 0.0
    for _ in range(500):
        c = random.uniform(*K.COXA_LIMIT)
        f = random.uniform(*K.FEMUR_LIMIT)
        t = random.uniform(*K.TIBIA_LIMIT)
        q = _q_from_legvec(c, f, t)
        for leg in K.LEGS:
            err = np.linalg.norm(u.link_pose(f"{leg.name}_foot", q)[:3, 3]
                                 - K.fk_leg_body(leg, c, f, t))
            max_err = max(max_err, err)
    print(f"{'OK  ' if max_err < 5e-5 else 'FAIL'} random FK vs model: max err {max_err:.2e} m")
    ok &= max_err < 5e-5

    print("\n" + ("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED"))
    return 0 if ok else 1

if __name__ == "__main__":
    raise SystemExit(main())
