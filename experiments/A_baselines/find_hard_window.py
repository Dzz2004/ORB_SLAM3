#!/usr/bin/env python3
"""Find the first hard window where all modes have connected keyframe metrics."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", required=True, type=Path)
    parser.add_argument("--a-root", required=True, type=Path)
    parser.add_argument("--difficulty", default="hard")
    parser.add_argument("--trajectory", default="P000")
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--seed", default=0, type=int)
    parser.add_argument("--frame-count", default=100, type=int)
    parser.add_argument("--step", default=50, type=int)
    args = parser.parse_args()

    repo = args.repo_root.resolve()
    a_root = args.a_root.resolve()
    output_root = args.output_root.resolve()
    sequence = (
        a_root
        / "data/TartanAirV2/OldTownSummer"
        / f"Data_{args.difficulty}"
        / args.trajectory
    )
    python_bin = a_root / "conda_envs/a_orb/bin/python"
    runner = a_root / "build/A_baselines/run_tartanair"
    vocab = repo / "Vocabulary/ORBvoc.txt"
    settings = repo / "configs/A/tartanair.yaml"
    prepare = repo / "experiments/A_baselines/prepare_sequence.py"
    run_script = repo / "experiments/A_baselines/run_baseline.py"
    evaluate = repo / "experiments/A_baselines/evaluate_run.py"
    modes = ("rgb", "rgb_imu", "rgbd_imu")

    camera_times = [
        float(line)
        for line in (sequence / "imu/cam_time.txt").read_text().splitlines()
        if line.strip()
    ]
    starts = range(
        0,
        len(camera_times) - args.frame_count + 1,
        args.step,
    )
    summary = []
    for start in starts:
        inputs = output_root / "inputs" / f"start_{start:04d}"
        subprocess.run(
            [
                str(python_bin),
                str(prepare),
                "--sequence-root",
                str(sequence),
                "--start-frame",
                str(start),
                "--num-frames",
                str(args.frame_count),
                "--output",
                str(inputs),
            ],
            check=True,
        )
        window = {"start_frame": start, "modes": {}}
        for mode in modes:
            run_dir = output_root / "runs" / f"start_{start:04d}" / mode
            command = [
                str(python_bin),
                str(run_script),
                "--binary",
                str(runner),
                "--vocab",
                str(vocab),
                "--settings",
                str(settings),
                "--mode",
                mode,
                "--manifest",
                str(inputs / "manifest.csv"),
                "--imu",
                str(inputs / "imu.csv"),
                "--output",
                str(run_dir),
                "--seed",
                str(args.seed),
                "--orb-root",
                str(repo),
            ]
            run_result = subprocess.run(command)
            alignment = "sim3" if mode == "rgb" else "se3"
            eval_result = subprocess.run(
                [
                    str(python_bin),
                    str(evaluate),
                    "--estimated",
                    str(run_dir / "trajectory_est.tum"),
                    "--reference",
                    str(inputs / "trajectory_gt.tum"),
                    "--tracking",
                    str(run_dir / "tracking.csv"),
                    "--output",
                    str(run_dir / "metrics.json"),
                    "--alignment",
                    alignment,
                ]
            )
            metrics_path = run_dir / "metrics.json"
            metrics = (
                json.loads(metrics_path.read_text())
                if metrics_path.exists()
                else {"status": "failed", "error": "metrics file missing"}
            )
            window["modes"][mode] = {
                "run_exit": run_result.returncode,
                "eval_exit": eval_result.returncode,
                "status": metrics.get("status"),
                "ate_rmse": metrics.get("ate_rmse"),
                "valid_pose_ratio": metrics.get("valid_pose_ratio"),
                "lost_events": metrics.get("lost_events"),
                "map_ids": metrics.get("map_ids_in_valid_tracking"),
            }
        summary.append(window)
        (output_root / "scan_results.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(window, sort_keys=True), flush=True)
        if all(
            window["modes"][mode]["status"] == "ok"
            and window["modes"][mode]["map_ids"]
            and len(window["modes"][mode]["map_ids"]) == 1
            for mode in modes
        ):
            print(f"SELECTED_START={start}", flush=True)
            return
    raise SystemExit("No hard window passed the selection rule")


if __name__ == "__main__":
    main()
