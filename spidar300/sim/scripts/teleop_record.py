from __future__ import annotations

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="SO-SPIDAR300-Climb-Play-v0")
parser.add_argument("--record", action="store_true")
parser.add_argument("--dataset_file", default="./data/so_spidar300_climb.hdf5")
parser.add_argument("--max_episodes", type=int, default=50)
parser.add_argument("--episode_timeout", type=float, default=None,
                    help="episode time-out in seconds (default: env config, 1800)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
_NO_CAM = os.environ.get("SPIDAR_NO_CAMERA", "") not in ("", "0")
if _NO_CAM and args.record:
    raise SystemExit("--record needs the camera; unset SPIDAR_NO_CAMERA")
args.enable_cameras = not _NO_CAM

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import numpy as np
import torch

import so_spidar300_lab.tasks
from so_spidar300_lab import kinematics as K
from so_spidar300_lab.teleop.keyboard_legged import HELP, KeyboardLeggedController
from isaaclab_tasks.utils import parse_env_cfg

class H5Recorder:

    CHUNK = 64

    def __init__(self, path: str, env_name: str, fps: float):
        import h5py
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self._f = h5py.File(path, "w")
        self._data = self._f.create_group("data")
        self._data.attrs["env_name"] = env_name
        self._data.attrs["fps"] = float(fps)
        self._n = 0
        self._ep = None
        self._dsets: dict = {}
        self._len = 0
        self._buf: list = []

    def add(self, action, state, rgb) -> None:
        self._buf.append((
            np.asarray(action, np.float32),
            np.asarray(state, np.float32),
            np.asarray(rgb, np.uint8),
        ))
        if len(self._buf) >= self.CHUNK:
            self._flush()

    def _flush(self) -> None:
        if not self._buf:
            return
        arrays = {
            "actions": np.stack([b[0] for b in self._buf]),
            "state": np.stack([b[1] for b in self._buf]),
            "rgb": np.stack([b[2] for b in self._buf]),
        }
        if self._ep is None:
            self._ep = self._data.create_group("demo_tmp")
            obs = self._ep.create_group("obs")
            parents = {"actions": self._ep, "state": obs, "rgb": obs}
            for name, arr in arrays.items():
                self._dsets[name] = parents[name].create_dataset(
                    name, data=arr, maxshape=(None,) + arr.shape[1:],
                    chunks=(self.CHUNK,) + arr.shape[1:],
                    compression="lzf" if name == "rgb" else None,
                )
        else:
            for name, arr in arrays.items():
                d = self._dsets[name]
                d.resize(d.shape[0] + arr.shape[0], axis=0)
                d[-arr.shape[0]:] = arr
        self._len += len(self._buf)
        self._buf = []

    def _reset_episode_state(self) -> None:
        self._ep = None
        self._dsets = {}
        self._len = 0
        self._buf = []

    def save_episode(self, success: bool) -> bool:
        self._flush()
        if self._ep is None:
            return False
        self._ep.attrs["num_samples"] = self._len
        self._ep.attrs["success"] = bool(success)
        self._f.move("data/demo_tmp", f"data/demo_{self._n}")
        self._f.flush()
        print(f"  записан demo_{self._n}: {self._len} кадров, success={success}")
        self._n += 1
        self._reset_episode_state()
        return True

    def discard_episode(self) -> None:
        had = self._len + len(self._buf)
        if self._ep is not None:
            del self._f["data/demo_tmp"]
        if had:
            print(f"  дубль выброшен ({had} кадров)")
        self._reset_episode_state()

    def close(self) -> None:
        self.discard_episode()
        self._data.attrs["num_demos"] = self._n
        self._f.close()

def main() -> None:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    if args.episode_timeout is not None:
        env_cfg.episode_length_s = args.episode_timeout
    env = gym.make(args.task, cfg=env_cfg).unwrapped
    print(f"[teleop] таймаут эпизода: {env_cfg.episode_length_s:.0f} с")

    teleop = KeyboardLeggedController(control_dt=env.step_dt)
    teleop.reset()

    fps = 1.0 / env.step_dt
    recorder = H5Recorder(args.dataset_file, args.task, fps) if args.record else None
    stance = K.stance_joint_vector()
    scale = float(env_cfg.actions.joints.scale)
    device = env.device

    flags = {"end": None}
    for key in ("ENTER", "NUMPAD_ENTER"):
        teleop.add_callback(key, lambda: flags.__setitem__("end", "save"))
    teleop.add_callback("BACKSPACE", lambda: flags.__setitem__("end", "discard"))

    obs, _ = env.reset()
    ep = 0
    print(HELP)
    print("  Эпизод: ENTER — сохранить и начать новый, BACKSPACE — выбросить дубль.")
    if recorder is not None:
        print(f"[recording] {fps:.0f} fps -> {args.dataset_file}\n")
    while simulation_app.is_running() and ep < args.max_episodes:
        target = teleop.advance()
        action = (target - stance) / scale
        action_t = torch.tensor(action, dtype=torch.float32, device=device).unsqueeze(0)

        obs, _, terminated, truncated, _ = env.step(action_t)

        if recorder is not None:
            state = obs["policy"][0].detach().cpu().numpy()
            rgb = (obs["rgb"]["image"][0].detach().cpu().numpy() * 255).astype(np.uint8)
            recorder.add(action, state, rgb)

        done = bool(terminated[0] or truncated[0])
        manual = flags["end"]
        flags["end"] = None
        if done:
            success = bool(env.success_buf[0].item())
            fired = [name for name in env.termination_manager.active_terms
                     if bool(env.termination_manager.get_term(name)[0])]
            print(f"[episode {ep}] reset, reason: {', '.join(fired) or 'unknown'} "
                  f"(success={success})")
            if recorder is not None:
                recorder.save_episode(success)
            teleop.reset()
            ep += 1
        elif manual is not None:
            if manual == "save":
                success = bool(env.success_buf[0].item())
                print(f"[episode {ep}] сохранение вручную (success={success})")
                if recorder is None or recorder.save_episode(success):
                    ep += 1
            else:
                print(f"[episode {ep}] дубль выброшен вручную")
                if recorder is not None:
                    recorder.discard_episode()
            obs, _ = env.reset()
            teleop.reset()

    if recorder is not None:
        recorder.close()
        print(f"saved {args.dataset_file}")
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
