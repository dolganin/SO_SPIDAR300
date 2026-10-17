from __future__ import annotations

import argparse
import re
from pathlib import Path

import coacd
import numpy as np
import trimesh

_URDF_DIR = Path(__file__).resolve().parents[1] / "urdf"

MIN_PART_VOLUME = 5e-6
CONVEX_RATIO = 2.0
MAX_HULL_VERTS = 32
PLASTIC_DENSITY = 1300.0
SERVO_MASS = 0.060
SERVO_VOLUME = 33.5e-6
SINGLE_HULL_LINKS = {"base_link"}

coacd.set_log_level("error")

def _parts(mesh: trimesh.Trimesh) -> list[trimesh.Trimesh]:
    out = []
    for body in mesh.split(only_watertight=False):
        if len(body.faces) < 4:
            continue
        try:
            hull = body.convex_hull
        except Exception:
            continue
        if hull.volume >= MIN_PART_VOLUME:
            out.append(body)
    return out

def _directions(n: int) -> np.ndarray:
    i = np.arange(n - 6) + 0.5
    z = 1 - 2 * i / (n - 6)
    phi = i * np.pi * (3 - np.sqrt(5))
    r = np.sqrt(1 - z * z)
    return np.vstack([np.eye(3), -np.eye(3), np.column_stack([r * np.cos(phi), r * np.sin(phi), z])])

def _simplify(points: np.ndarray) -> trimesh.Trimesh:
    hull = trimesh.convex.convex_hull(points)
    if len(hull.vertices) <= MAX_HULL_VERTS:
        return hull
    keep = np.unique(np.argmax(hull.vertices @ _directions(MAX_HULL_VERTS).T, axis=0))
    return trimesh.convex.convex_hull(hull.vertices[keep])

def _decompose(body: trimesh.Trimesh, threshold: float, max_hulls: int) -> list[trimesh.Trimesh]:
    pieces = coacd.run_coacd(coacd.Mesh(body.vertices, body.faces), threshold=threshold,
                             max_convex_hull=max_hulls, seed=0)
    return [_simplify(v) for v, _ in pieces]

def collision_hulls(name: str, mesh: trimesh.Trimesh, threshold: float, max_hulls: int) -> list[trimesh.Trimesh]:
    parts = _parts(mesh)
    if name in SINGLE_HULL_LINKS:
        return [_simplify(np.vstack([p.vertices for p in parts]))]
    hulls = []
    for body in parts:
        if body.convex_hull.volume / max(abs(body.volume), 1e-12) <= CONVEX_RATIO:
            hulls.append(_simplify(body.vertices))
        else:
            hulls.extend(_decompose(body, threshold, max_hulls))
    return sorted(hulls, key=lambda h: -h.volume)

def _inertial_xml(mesh: trimesh.Trimesh) -> str:
    servos, plastic = [], []
    for body in mesh.split(only_watertight=False):
        is_servo = body.is_watertight and abs(body.volume / SERVO_VOLUME - 1.0) < 0.05
        (servos if is_servo else plastic).append(body)
    solids = []
    for body in servos:
        body.density = SERVO_MASS / body.volume
        solids.append(body)
    if plastic:
        rest = trimesh.util.concatenate(plastic)
        rest.density = PLASTIC_DENSITY
        solids.append(rest)
    mass = sum(b.mass for b in solids)
    com = sum(b.mass * b.center_mass for b in solids) / mass
    i = np.zeros((3, 3))
    for b in solids:
        d = b.center_mass - com
        i += b.moment_inertia + b.mass * (d @ d * np.eye(3) - np.outer(d, d))
    return (f'    <inertial>\n'
            f'      <origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" rpy="0 0 0"/>\n'
            f'      <mass value="{mass:.4f}"/>\n'
            f'      <inertia ixx="{i[0, 0]:.4e}" ixy="{i[0, 1]:.4e}" ixz="{i[0, 2]:.4e}" '
            f'iyy="{i[1, 1]:.4e}" iyz="{i[1, 2]:.4e}" izz="{i[2, 2]:.4e}"/>\n'
            f'    </inertial>\n')

def _collision_xml(files: list[str]) -> str:
    return "\n".join(
        f'    <collision>\n'
        f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        f'      <geometry>\n'
        f'        <mesh filename="{f}"/>\n'
        f'      </geometry>\n'
        f'    </collision>' for f in files)

_LINK_RE = re.compile(r'(  <link name="([^"]+)">\n)(.*?)(  </link>)', re.S)
_INERTIAL_RE = re.compile(r'    <inertial>.*</inertial>\n', re.S)
_COLLISION_RE = re.compile(r'    <collision>.*</collision>\n', re.S)
_VISUAL_MESH_RE = re.compile(r'<visual>.*?<mesh filename="([^"]+)"', re.S)

def main() -> None:
    parser = argparse.ArgumentParser(description="Generate convex collision meshes and inertials for urdf/robot.urdf")
    parser.add_argument("--urdf", default=str(_URDF_DIR / "robot.urdf"))
    parser.add_argument("--threshold", type=float, default=0.1, help="CoACD concavity threshold")
    parser.add_argument("--max-hulls", type=int, default=6, help="Max hulls per decomposed part")
    args = parser.parse_args()

    urdf = Path(args.urdf).resolve()
    out_dir = urdf.parent / "meshes" / "collision"
    out_dir.mkdir(parents=True, exist_ok=True)
    for old in out_dir.glob("*.stl"):
        old.unlink()

    text = urdf.read_text()
    files_by_link: dict[str, list[str]] = {}
    inertial_by_link: dict[str, str] = {}
    for _, name, body, _ in _LINK_RE.findall(text):
        visual = _VISUAL_MESH_RE.search(body)
        if not visual:
            continue
        mesh = trimesh.load(urdf.parent / visual.group(1), force="mesh")
        hulls = collision_hulls(name, mesh, args.threshold, args.max_hulls)
        files = []
        for i, hull in enumerate(hulls):
            rel = f"meshes/collision/{name}_{i}.stl"
            hull.export(urdf.parent / rel)
            files.append(str(urdf.parent / rel))
        files_by_link[name] = files
        inertial_by_link[name] = _inertial_xml(mesh)
        print(f"{name:14s} {len(mesh.faces):6d} tris -> {len(hulls)} hulls, "
              f"{sum(len(h.faces) for h in hulls):4d} tris")

    def with_collision(m: re.Match) -> str:
        files = files_by_link.get(m.group(2))
        if not files:
            return m.group(0)
        body = _COLLISION_RE.sub("", _INERTIAL_RE.sub("", m.group(3))) + _collision_xml(files) + "\n"
        return m.group(1) + inertial_by_link[m.group(2)] + body + m.group(4)

    text = _LINK_RE.sub(with_collision, text)
    urdf.write_text(text)

    print("written:", urdf)

if __name__ == "__main__":
    main()
