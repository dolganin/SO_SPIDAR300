from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
_PACKAGE_SRC = _ROOT


DEFAULT_HDF5 = "./data/so_spidar300_climb.hdf5"
DEFAULT_REPO_ID = "80n3yB4dg3r/SPIDAR300"


def _env() -> dict[str, str]:
    env = os.environ.copy()
    old = env.get("PYTHONPATH")
    env["PYTHONPATH"] = str(_PACKAGE_SRC) if not old else f"{_PACKAGE_SRC}{os.pathsep}{old}"
    return env


def _run(cmd: list[str], dry_run: bool = False) -> int:
    print("\n$ " + " ".join(cmd))
    if dry_run:
        return 0
    return subprocess.call(cmd, cwd=_ROOT, env=_env())


def collect(args: argparse.Namespace) -> int:
    cmd = [
        sys.executable,
        str(_HERE / "teleop_record.py"),
        "--record",
        "--dataset_file",
        args.dataset_file,
        "--max_episodes",
        str(args.max_episodes),
        "--task",
        args.task,
    ]
    if args.episode_timeout is not None:
        cmd += ["--episode_timeout", str(args.episode_timeout)]
    cmd.extend(args.sim_arg)
    return _run(cmd, args.dry_run)


def convert_asset(args: argparse.Namespace) -> int:
    cmd = [sys.executable, str(_HERE / "convert_urdf_to_usd.py")]
    cmd.extend(args.sim_arg)
    return _run(cmd, args.dry_run)


def convert(args: argparse.Namespace) -> int:
    cmd = [
        sys.executable,
        "-m",
        "so_spidar300_lab.conversion.hdf5_to_lerobot",
        "--hdf5",
        args.dataset_file,
        "--repo-id",
        args.repo_id,
    ]
    if args.dataset_root:
        cmd += ["--root", args.dataset_root]
    if args.fps is not None:
        cmd += ["--fps", str(args.fps)]
    if args.only_success:
        cmd.append("--only-success")
    return _run(cmd, args.dry_run)


def train(args: argparse.Namespace) -> int:
    cmd = [
        sys.executable,
        str(_HERE / "train_lerobot.py"),
        "--dataset-repo-id",
        args.repo_id,
        "--policy",
        args.policy,
        "--output-dir",
        args.output_dir,
        "--job-name",
        args.job_name,
        "--device",
        args.device,
    ]
    if args.dataset_root:
        cmd += ["--dataset-root", args.dataset_root]
    if args.steps is not None:
        cmd += ["--steps", str(args.steps)]
    if args.batch_size is not None:
        cmd += ["--batch-size", str(args.batch_size)]
    if args.wandb:
        cmd.append("--wandb")
    if args.resume:
        cmd.append("--resume")
    for item in args.train_arg:
        cmd.append(f"--train-arg={item}")
    if args.dry_run:
        cmd.append("--dry-run")
    return _run(cmd, False)


def run_all(args: argparse.Namespace) -> int:
    steps = [collect, convert, train]
    if not args.skip_asset_convert:
        steps.insert(0, convert_asset)
    for step in steps:
        code = step(args)
        if code != 0:
            return code
    return 0


def _add_common_data_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--dataset-file", default=DEFAULT_HDF5)
    parser.add_argument("--repo-id", default=DEFAULT_REPO_ID)
    parser.add_argument("--dataset-root", default=None)
    parser.add_argument("--dry-run", action="store_true")


def _add_collect_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--task", default="SO-SPIDAR300-Climb-Play-v0")
    parser.add_argument("--max-episodes", type=int, default=50)
    parser.add_argument("--episode-timeout", type=float, default=None)
    parser.add_argument(
        "--sim-arg",
        action="append",
        default=[],
        help="Extra Isaac AppLauncher argument. Use --sim-arg=--headless for dashed values.",
    )


def _add_convert_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--fps", type=int, default=None)
    parser.add_argument("--only-success", action="store_true")


def _add_train_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--policy", default="act")
    parser.add_argument("--output-dir", default="./outputs/train/so_spidar300_act")
    parser.add_argument("--job-name", default="so_spidar300_act")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--batch-size", type=int, default=None)
    parser.add_argument("--wandb", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument(
        "--train-arg",
        action="append",
        default=[],
        help="Extra raw LeRobot/Hydra argument. Use --train-arg=--steps=100000 for dashed values.",
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SO-SPIDAR300 data collection, conversion, and training pipeline."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_asset = sub.add_parser("asset", help="Convert current URDF robot asset to USD.")
    p_asset.add_argument("--dry-run", action="store_true")
    p_asset.add_argument(
        "--sim-arg",
        action="append",
        default=[],
        help="Extra Isaac AppLauncher argument. Use --sim-arg=--headless for dashed values.",
    )
    p_asset.set_defaults(func=convert_asset)

    p_collect = sub.add_parser("collect", help="Record teleop demonstrations to HDF5.")
    _add_common_data_args(p_collect)
    _add_collect_args(p_collect)
    p_collect.set_defaults(func=collect)

    p_convert = sub.add_parser("convert", help="Convert HDF5 demos to LeRobotDataset.")
    _add_common_data_args(p_convert)
    _add_convert_args(p_convert)
    p_convert.set_defaults(func=convert)

    p_train = sub.add_parser("train", help="Train a LeRobot policy.")
    _add_common_data_args(p_train)
    _add_train_args(p_train)
    p_train.set_defaults(func=train)

    p_all = sub.add_parser("all", help="Run collect, convert, then train.")
    _add_common_data_args(p_all)
    _add_collect_args(p_all)
    _add_convert_args(p_all)
    _add_train_args(p_all)
    p_all.add_argument(
        "--skip-asset-convert",
        action="store_true",
        help="Do not regenerate so_spidar300.usd from the current URDF before collection.",
    )
    p_all.set_defaults(func=run_all)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
