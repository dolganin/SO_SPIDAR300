from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="SO-SPIDAR300-Climb-Play-v0")
parser.add_argument("--dataset_file", required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import h5py
import torch

import so_spidar300_lab.tasks
from isaaclab_tasks.utils import parse_env_cfg

def main() -> None:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    env = gym.make(args.task, cfg=env_cfg).unwrapped
    env.reset()

    with h5py.File(args.dataset_file, "r") as f:
        demos = sorted(f["data"].keys(), key=lambda k: int(k.split("_")[-1]))
        for name in demos:
            actions = f["data"][name]["actions"][:]
            success = bool(f["data"][name].attrs.get("success", False))
            print(f"replaying {name} (success={success}, {len(actions)} steps)")
            env.reset()
            for a in actions:
                if not simulation_app.is_running():
                    break
                env.step(torch.tensor(a, dtype=torch.float32, device=env.device).unsqueeze(0))
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
