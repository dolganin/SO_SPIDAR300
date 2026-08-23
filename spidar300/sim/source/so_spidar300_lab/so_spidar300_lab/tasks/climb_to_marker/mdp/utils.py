from __future__ import annotations

import torch

def ensure_goal_buffers(env) -> None:
    if hasattr(env, "goal_pos_w"):
        return
    n, dev = env.num_envs, env.device
    env.goal_pos_w = torch.zeros(n, 3, device=dev)
    env.onback_timer = torch.zeros(n, device=dev)
    env.success_buf = torch.zeros(n, dtype=torch.bool, device=dev)
    env.prev_goal_dist = torch.zeros(n, device=dev)
