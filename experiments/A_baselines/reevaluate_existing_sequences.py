#!/usr/bin/env python3
"""Re-evaluate existing A runs without rerunning ORB-SLAM3.

This repairs evaluation-only defects from the earlier matrix while preserving the
original engine outputs. Existing metrics are copied to metrics.pre_evaluator_fix.json.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

import yaml


MODES = ("rgb", "rgb_imu", "rgbd_imu")
SEEDS = (0, 1, 2)


def row_count(path: Path) -> int:
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(encoding="utf-8") as handle:
        return sum(1 for line in handle if line.strip())


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "Usage: reevaluate_existing_sequences.py ROOT MATRIX PYTHON",
            file=sys.stderr,
        )
        return 2
    root = Path(sys.argv[1]).resolve()
    matrix_path = Path(sys.argv[2]).resolve()
    python = sys.argv[3]
    matrix = yaml.safe_load(matrix_path.read_text(encoding="utf-8"))
    failures = 0
    evaluated = 0
    for sequence in matrix["sequences"]:
        run_root = root / sequence["result_root"]
        for mode in MODES:
            for seed in SEEDS:
                run_dir = run_root / f"{mode}_seed{seed}"
                tracking = run_dir / "tracking.csv"
                if not tracking.exists():
                    print(f"MISSING {run_dir}")
                    failures += 1
                    continue
                metadata_path = run_dir / "run_meta.json"
                metadata = (
                    json.loads(metadata_path.read_text(encoding="utf-8"))
                    if metadata_path.exists()
                    else {}
                )
                manifest = metadata.get("command", [None] * 5)[4]
                if manifest:
                    inputs = Path(manifest).resolve().parent
                elif sequence.get("input_root"):
                    inputs = root / sequence["input_root"]
                else:
                    inputs = (
                        root
                        / "results/A/full_sequences/inputs"
                        / sequence["id"]
                    )
                metrics = run_dir / "metrics.json"
                backup = run_dir / "metrics.pre_evaluator_fix.json"
                if metrics.exists() and not backup.exists():
                    shutil.copy2(metrics, backup)
                alignment = "sim3" if mode == "rgb" else "se3"
                command = [
                    python,
                    str(root / "experiments/A_baselines/evaluate_run.py"),
                    "--estimated",
                    str(run_dir / "trajectory_est.tum"),
                    "--reference",
                    str(inputs / "trajectory_gt.tum"),
                    "--tracking",
                    str(tracking),
                    "--output",
                    str(metrics),
                    "--alignment",
                    alignment,
                    "--plot",
                    str(run_dir / "trajectory_overlay.png"),
                ]
                completed = subprocess.run(
                    command,
                    cwd=root,
                    text=True,
                    capture_output=True,
                )
                evaluated += 1
                result = json.loads(metrics.read_text(encoding="utf-8"))
                if metadata_path.exists():
                    keyframe_count = row_count(
                        run_dir / "keyframe_trajectory.tum"
                    )
                    frame_pose_count = row_count(
                        run_dir / "trajectory_frames.tum"
                    )
                    metadata["keyframe_count"] = keyframe_count
                    metadata["frame_pose_count"] = frame_pose_count
                    metadata["primary_trajectory_row_count"] = row_count(
                        run_dir / "trajectory_est.tum"
                    )
                    if metadata["primary_trajectory_row_count"] > 0:
                        source = "optimized_keyframe_body"
                    elif keyframe_count == 0:
                        source = "unavailable_empty_keyframe"
                    elif keyframe_count < 3:
                        source = "unavailable_sparse_keyframe"
                    else:
                        source = "unavailable_degenerate_keyframe"
                    metadata["trajectory_source"] = source
                    metadata["optimized_trajectory_source"] = source
                    metadata_path.write_text(
                        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                print(
                    f"{sequence['id']} {mode} seed{seed}: "
                    f"{result.get('status')} "
                    f"ate={result.get('ate_status')} "
                    f"rpe={result.get('rpe_translation_status')}"
                )
                if completed.returncode != 0:
                    print(completed.stderr, file=sys.stderr)
                    failures += 1
    print(f"evaluated={evaluated} failures={failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
