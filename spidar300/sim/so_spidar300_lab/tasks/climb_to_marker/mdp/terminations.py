from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from .utils import ensure_goal_buffers

if TYPE_CHECKING:
    from ..climb_env import SpidarClimbEnv

def reached_goal(env: "SpidarClimbEnv",
                 eps: tuple[float, float, float] = (0.06, 0.06, 0.06),
                 asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    ensure_goal_buffers(env)
    asset: Articulation = env.scene[asset_cfg.name]
    err = (env.goal_pos_w - asset.data.root_pos_w).abs()
    eps_t = torch.tensor(eps, device=err.device)
    success = (err < eps_t).all(dim=-1)
    env.success_buf = success
    return success

def on_back_timeout(env: "SpidarClimbEnv", time_s: float = 5.0,
                    speed_thresh: float = 0.05,
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    ensure_goal_buffers(env)
    asset: Articulation = env.scene[asset_cfg.name]
    on_back = asset.data.projected_gravity_b[:, 2] > 0.0
    speed = torch.norm(asset.data.root_lin_vel_w, dim=-1)
    stuck = on_back & (speed < speed_thresh)
    env.onback_timer = torch.where(stuck, env.onback_timer + env.step_dt,
                                   torch.zeros_like(env.onback_timer))
    return env.onback_timer > time_s

def base_too_low(env: "SpidarClimbEnv", min_height: float = -0.5,
                 asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    rel_h = asset.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    return rel_h < min_height
