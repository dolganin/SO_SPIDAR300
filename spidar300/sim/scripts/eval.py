from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="SO-SPIDAR300-Climb-Play-v0")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--checkpoint", help="local LeRobot pretrained_model directory")
group.add_argument("--model-repo-id", help="Hugging Face model repository, e.g. 80n3yB4dg3r/SPIDAR300-ACT")
parser.add_argument("--episodes", type=int, default=20)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from huggingface_hub import snapshot_download

import so_spidar300_lab.tasks
from isaaclab_tasks.utils import parse_env_cfg
from lerobot.common.policies.act.modeling_act import ACTPolicy

def main() -> None:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    env = gym.make(args.task, cfg=env_cfg).unwrapped
    device = env.device

    checkpoint = args.checkpoint or snapshot_download(repo_id=args.model_repo_id)
    policy = ACTPolicy.from_pretrained(checkpoint).to(device).eval()

    successes = 0
    on_back_fails = 0
    for ep in range(args.episodes):
        obs, _ = env.reset()
        policy.reset()
        done = False
        while simulation_app.is_running() and not done:
            batch = {
                "observation.images.front": obs["rgb"]["image"].permute(0, 3, 1, 2),
                "observation.state": obs["policy"],
            }
            with torch.inference_mode():
                action = policy.select_action(batch)
            obs, _, terminated, truncated, _ = env.step(action)
            done = bool(terminated[0] or truncated[0])
        success = bool(env.success_buf[0].item())
        on_back = bool(env.onback_timer[0].item() > 5.0)
        successes += success
        on_back_fails += on_back
        print(f"episode {ep}: success={success} on_back_fail={on_back}")

    n = args.episodes
    print(f"\nsuccess rate: {successes}/{n} = {successes / n:.1%}")
    print(f"on-back failures: {on_back_fails}/{n}")
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
