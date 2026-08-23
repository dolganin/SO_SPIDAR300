from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from .utils import ensure_goal_buffers

if TYPE_CHECKING:
    from ..climb_env import SpidarClimbEnv

def _goal_distance(env: "SpidarClimbEnv", asset: Articulation) -> torch.Tensor:
    return torch.norm(env.goal_pos_w - asset.data.root_pos_w, dim=-1)

def progress_to_goal(env: "SpidarClimbEnv",
                     asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    ensure_goal_buffers(env)
    asset: Articulation = env.scene[asset_cfg.name]
    dist = _goal_distance(env, asset)
    reward = env.prev_goal_dist - dist
    env.prev_goal_dist = dist.detach().clone()
    reward = torch.where(env.episode_length_buf <= 1, torch.zeros_like(reward), reward)
    return reward

def goal_height_gain(env: "SpidarClimbEnv",
                     asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    ensure_goal_buffers(env)
    asset: Articulation = env.scene[asset_cfg.name]
    dz = env.goal_pos_w[:, 2] - asset.data.root_pos_w[:, 2]
    return torch.clamp(dz, min=0.0)

def reached_goal_bonus(env: "SpidarClimbEnv", eps: float = 0.08,
                       asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    ensure_goal_buffers(env)
    asset: Articulation = env.scene[asset_cfg.name]
    return (_goal_distance(env, asset) < eps).float()

def upright(env: "SpidarClimbEnv",
            asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return -asset.data.projected_gravity_b[:, 2]

def on_back_penalty(env: "SpidarClimbEnv",
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return (asset.data.projected_gravity_b[:, 2] > 0.0).float()
