from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
_PACKAGE_SRC = _ROOT / "source" / "so_spidar300_lab"


def _env() -> dict[str, str]:
    env = os.environ.copy()
    old = env.get("PYTHONPATH")
    env["PYTHONPATH"] = str(_PACKAGE_SRC) if not old else f"{_PACKAGE_SRC}{os.pathsep}{old}"
    return env


def _auto_entrypoint() -> list[str]:
    exe = shutil.which("lerobot-train")
    if exe:
        return [exe]
    return [sys.executable, "-m", "lerobot.scripts.train"]


def build_command(args: argparse.Namespace) -> list[str]:
    if args.entrypoint:
        cmd = args.entrypoint
    else:
        cmd = _auto_entrypoint()

    train_args = [
        f"--dataset.repo_id={args.dataset_repo_id}",
        f"--policy.type={args.policy}",
        f"--output_dir={args.output_dir}",
        f"--job_name={args.job_name}",
        f"--policy.device={args.device}",
        f"--wandb.enable={'true' if args.wandb else 'false'}",
    ]
    if args.dataset_root:
        train_args.append(f"--dataset.root={args.dataset_root}")
    if args.steps is not None:
        train_args.append(f"--steps={args.steps}")
    if args.batch_size is not None:
        train_args.append(f"--batch_size={args.batch_size}")
    if args.resume:
        train_args.append("--resume=true")
    train_args.extend(args.train_arg)
    return cmd + train_args


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Launch LeRobot imitation training for SO-SPIDAR300 datasets."
    )
    parser.add_argument("--dataset-repo-id", default="80n3yB4dg3r/SPIDAR300")
    parser.add_argument("--dataset-root", default=None)
    parser.add_argument("--policy", default="act", help="LeRobot policy type, e.g. act")
    parser.add_argument("--output-dir", default="./outputs/train/so_spidar300_act")
    parser.add_argument("--job-name", default="so_spidar300_act")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--batch-size", type=int, default=None)
    parser.add_argument("--wandb", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument(
        "--entrypoint",
        nargs="+",
        default=None,
        help="Override training entrypoint, e.g. --entrypoint python -m lerobot.scripts.train",
    )
    parser.add_argument(
        "--train-arg",
        action="append",
        default=[],
        help="Extra raw LeRobot/Hydra argument. Use --train-arg=--steps=100000 for dashed values.",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    cmd = build_command(args)
    print(" ".join(cmd))
    if args.dry_run:
        return 0
    return subprocess.call(cmd, cwd=_ROOT, env=_env())


if __name__ == "__main__":
    raise SystemExit(main())
