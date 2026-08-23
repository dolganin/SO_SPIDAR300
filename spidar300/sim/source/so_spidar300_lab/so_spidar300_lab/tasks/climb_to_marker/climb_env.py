from __future__ import annotations

import torch

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.markers import VisualizationMarkers

from .climb_env_cfg import SpidarClimbEnvCfg
from .mdp.utils import ensure_goal_buffers

class SpidarClimbEnv(ManagerBasedRLEnv):
    cfg: SpidarClimbEnvCfg

    def __init__(self, cfg: SpidarClimbEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        ensure_goal_buffers(self)
        self._goal_marker = VisualizationMarkers(self.cfg.goal_marker)
        self._resample_goals(torch.arange(self.num_envs, device=self.device))

    def _resample_goals(self, env_ids: torch.Tensor) -> None:
        n = len(env_ids)
        dev = self.device
        base = torch.tensor(self.cfg.goal_offset, device=dev)
        rand = torch.tensor(self.cfg.goal_randomization, device=dev)
        jitter = (torch.rand(n, 3, device=dev) * 2.0 - 1.0) * rand
        self.goal_pos_w[env_ids] = self.scene.env_origins[env_ids] + base + jitter
        self._goal_marker.visualize(translations=self.goal_pos_w)

    def _reset_idx(self, env_ids: torch.Tensor) -> None:
        super()._reset_idx(env_ids)
        if not hasattr(self, "_goal_marker"):
            return
        self.onback_timer[env_ids] = 0.0
        self.success_buf[env_ids] = False
        self._resample_goals(env_ids)
        robot = self.scene["robot"]
        self.prev_goal_dist[env_ids] = torch.norm(
            self.goal_pos_w[env_ids] - robot.data.root_pos_w[env_ids], dim=-1
        )
