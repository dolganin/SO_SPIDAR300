from __future__ import annotations

import os
import sys
from typing import Iterator

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, "..", "..", "..")))
from so_spidar300_lab import kinematics as K

ACTION_DIM = len(K.JOINT_NAMES_18)

def build_features(image_hw: tuple[int, int], state_dim: int) -> dict:
    h, w = image_hw
    return {
        "observation.images.front": {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": [f"s{i}" for i in range(state_dim)],
        },
        "action": {
            "dtype": "float32",
            "shape": (ACTION_DIM,),
            "names": list(K.JOINT_NAMES_18),
        },
        "success": {"dtype": "float32", "shape": (1,), "names": ["success"]},
    }

def episode_to_frames(ep: dict) -> list[dict]:
    actions = np.asarray(ep["actions"], dtype=np.float32)
    state = np.asarray(ep["state"], dtype=np.float32)
    rgb = np.asarray(ep["rgb"])
    t = actions.shape[0]
    if not (state.shape[0] == rgb.shape[0] == t):
        raise ValueError(f"length mismatch: actions={t} state={state.shape[0]} rgb={rgb.shape[0]}")
    if actions.shape[1] != ACTION_DIM:
        raise ValueError(f"action dim {actions.shape[1]} != {ACTION_DIM}")
    if rgb.ndim != 4 or rgb.shape[-1] != 3:
        raise ValueError(f"rgb must be (T,H,W,3), got {rgb.shape}")
    success = np.float32(1.0 if ep.get("success", False) else 0.0)
    frames = []
    for i in range(t):
        frames.append({
            "observation.images.front": rgb[i],
            "observation.state": state[i],
            "action": actions[i],
            "success": np.array([success], dtype=np.float32),
        })
    return frames

def read_meta(h5_path: str) -> dict:
    import h5py
    with h5py.File(h5_path, "r") as f:
        return dict(f["data"].attrs)

def iter_episodes(h5_path: str) -> Iterator[dict]:
    import h5py
    with h5py.File(h5_path, "r") as f:
        data = f["data"]
        demos = sorted((k for k in data.keys() if k != "demo_tmp"),
                       key=lambda k: int(k.split("_")[-1]))
        for name in demos:
            g = data[name]
            yield {
                "actions": g["actions"][:],
                "state": g["obs"]["state"][:],
                "rgb": g["obs"]["rgb"][:],
                "success": bool(g.attrs.get("success", False)),
            }

def convert(h5_path: str, repo_id: str, fps: int | None = None,
            root: str | None = None, only_success: bool = False) -> None:
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    if fps is None:
        fps = int(round(read_meta(h5_path).get("fps", 30.0)))

    it = iter_episodes(h5_path)
    first = next(it, None)
    if first is None:
        raise RuntimeError(f"no episodes found in {h5_path}")
    h, w = first["rgb"].shape[1:3]
    state_dim = first["state"].shape[1]
    features = build_features((h, w), state_dim)

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        root=root,
        robot_type="so_spidar300",
        use_videos=True,
    )
    n_written = n_total = 0
    import itertools
    for ep in itertools.chain([first], it):
        n_total += 1
        if only_success and not ep["success"]:
            continue
        if ep["actions"].shape[0] == 0:
            continue
        for frame in episode_to_frames(ep):
            dataset.add_frame(frame, task="climb to the marker")
        dataset.save_episode()
        n_written += 1
    print(f"wrote {n_written}/{n_total} episodes to LeRobot dataset '{repo_id}' (fps={fps})")

def main() -> None:
    import argparse
    p = argparse.ArgumentParser(description="HDF5 -> LeRobotDataset for SO-SPIDAR300")
    p.add_argument("--hdf5", required=True, help="input demo HDF5 file")
    p.add_argument("--repo-id", default="80n3yB4dg3r/SPIDAR300")
    p.add_argument("--fps", type=int, default=None,
                   help="override; default = fps recorded in the HDF5 (30)")
    p.add_argument("--root", default=None, help="dataset root (default: HF cache)")
    p.add_argument("--only-success", action="store_true", help="keep only successful episodes")
    a = p.parse_args()
    convert(a.hdf5, a.repo_id, a.fps, a.root, a.only_success)

if __name__ == "__main__":
    main()
