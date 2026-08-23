from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import TiledCamera
try:
    from isaaclab.utils.math import quat_apply_inverse
except ImportError:
    from isaaclab.utils.math import quat_rotate_inverse as quat_apply_inverse

from .utils import ensure_goal_buffers

if TYPE_CHECKING:
    from ..climb_env import SpidarClimbEnv

def target_vector_b(env: "SpidarClimbEnv",
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    ensure_goal_buffers(env)
    asset: Articulation = env.scene[asset_cfg.name]
    delta_w = env.goal_pos_w - asset.data.root_pos_w
    return quat_apply_inverse(asset.data.root_quat_w, delta_w)

def target_distance(env: "SpidarClimbEnv",
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    ensure_goal_buffers(env)
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.norm(env.goal_pos_w - asset.data.root_pos_w, dim=-1, keepdim=True)

def is_on_back(env: "SpidarClimbEnv",
               asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return (asset.data.projected_gravity_b[:, 2] > 0.0).float().unsqueeze(-1)

def camera_rgb(env: "SpidarClimbEnv",
               sensor_cfg: SceneEntityCfg = SceneEntityCfg("camera")) -> torch.Tensor:
    sensor: TiledCamera = env.scene[sensor_cfg.name]
    rgb = sensor.data.output["rgb"].float()
    if rgb.shape[-1] == 4:
        rgb = rgb[..., :3]
    return rgb / 255.0
