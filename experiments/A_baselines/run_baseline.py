#!/usr/bin/env python3
"""Execute one headless ORB-SLAM3 TartanAir baseline run."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def git_head(repo: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def trajectory_row_count(path: Path) -> int:
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(encoding="utf-8") as handle:
        return sum(1 for line in handle if line.strip())


def imu_metadata(manifest: Path, imu: Path) -> dict[str, float | int]:
    with manifest.open(newline="", encoding="utf-8") as handle:
        first_frame = next(csv.DictReader(handle))
    first_timestamp = float(first_frame["timestamp"])
    timestamps: list[float] = []
    with imu.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            timestamps.append(float(row["timestamp"]))
    before_or_at = [stamp for stamp in timestamps if stamp <= first_timestamp]
    return {
        "first_frame_timestamp": first_timestamp,
        "imu_first_timestamp": timestamps[0] if timestamps else math.nan,
        "imu_last_timestamp": timestamps[-1] if timestamps else math.nan,
        "imu_samples_before_or_at_first_frame": len(before_or_at),
        "imu_sample_count": len(timestamps),
    }


def materialize_orb_trajectory(output: Path, settings: Path) -> dict[str, object]:
    primary_path = output / "trajectory_est.tum"
    frame_pose_count = trajectory_row_count(primary_path)
    if primary_path.exists():
        shutil.copy2(primary_path, output / "trajectory_frames.tum")
    keyframe_path = output / "keyframe_trajectory.tum"
    keyframe_count = trajectory_row_count(keyframe_path)
    if not keyframe_path.exists() or keyframe_path.stat().st_size == 0:
        primary_path.write_text("", encoding="utf-8")
        return {
            "primary": "unavailable_empty_keyframe",
            "optimized": "unavailable_empty_keyframe",
            "keyframe_count": keyframe_count,
            "frame_pose_count": frame_pose_count,
            "covariance_rank": None,
        }
    if keyframe_count == 0:
        primary_path.write_text("", encoding="utf-8")
        return {
            "primary": "unavailable_empty_keyframe",
            "optimized": "unavailable_empty_keyframe",
            "keyframe_count": keyframe_count,
            "frame_pose_count": frame_pose_count,
            "covariance_rank": None,
        }
    rows = np.loadtxt(keyframe_path, ndmin=2)
    if keyframe_count < 3:
        primary_path.write_text("", encoding="utf-8")
        return {
            "primary": "unavailable_sparse_keyframe",
            "optimized": "unavailable_sparse_keyframe",
            "keyframe_count": keyframe_count,
            "frame_pose_count": frame_pose_count,
            "covariance_rank": None,
        }
    covariance_rank = np.linalg.matrix_rank(
        np.cov(rows[:, 1:4], rowvar=False)
    )
    if covariance_rank < 2:
        primary_path.write_text("", encoding="utf-8")
        return {
            "primary": "unavailable_degenerate_keyframe",
            "optimized": "unavailable_degenerate_keyframe",
            "keyframe_count": keyframe_count,
            "frame_pose_count": frame_pose_count,
            "covariance_rank": int(covariance_rank),
        }

    storage = cv2.FileStorage(str(settings), cv2.FILE_STORAGE_READ)
    tbc = storage.getNode("IMU.T_b_c1").mat()
    storage.release()
    tcb_rotation = np.linalg.inv(tbc[:3, :3])
    camera_rotation = Rotation.from_quat(rows[:, [4, 5, 6, 7]])
    body_rotation = camera_rotation * Rotation.from_matrix(tcb_rotation)
    rows[:, 3:7] = body_rotation.as_quat()[:, [3, 0, 1, 2]]

    np.savetxt(
        primary_path,
        rows,
        fmt="%.17g",
        delimiter=" ",
    )
    shutil.copy2(primary_path, output / "trajectory_optimized.tum")
    return {
        "primary": "optimized_keyframe_body",
        "optimized": "keyframe_body",
        "keyframe_count": keyframe_count,
        "frame_pose_count": frame_pose_count,
        "covariance_rank": int(covariance_rank),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--vocab", required=True, type=Path)
    parser.add_argument("--settings", required=True, type=Path)
    parser.add_argument(
        "--mode", required=True, choices=["rgb", "rgb_imu", "rgbd_imu"]
    )
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--imu", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seed", default=0, type=int)
    parser.add_argument("--orb-root", required=True, type=Path)
    args = parser.parse_args()

    output = args.output.resolve()
    if output.exists():
        raise SystemExit(f"Refusing to overwrite existing run directory: {output}")
    output.mkdir(parents=True)

    shutil.copy2(args.settings, output / "settings_snapshot.yaml")
    shutil.copy2(args.manifest, output / "manifest_snapshot.csv")
    shutil.copy2(args.imu, output / "imu_snapshot.csv")

    command = [
        str(args.binary.resolve()),
        str(args.vocab.resolve()),
        str(args.settings.resolve()),
        args.mode,
        str(args.manifest.resolve()),
        str(args.imu.resolve()),
        str(output),
        str(args.seed),
    ]
    meta = {
        "command": command,
        "mode": args.mode,
        "seed": args.seed,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "cwd": str(Path.cwd()),
        "git_commit": git_head(args.orb_root),
        "environment": {
            key: os.environ.get(key)
            for key in [
                "CONDA_PREFIX",
                "CMAKE_PREFIX_PATH",
                "LD_LIBRARY_PATH",
                "CPATH",
                "TMPDIR",
            ]
        },
    }

    with (output / "run.log").open("w", encoding="utf-8") as log:
        log.write("COMMAND=" + json.dumps(command) + "\n")
        log.flush()
        run_env = os.environ.copy()
        run_env.update(
            {
                "OMP_NUM_THREADS": "1",
                "OPENBLAS_NUM_THREADS": "1",
                "MKL_NUM_THREADS": "1",
            }
        )
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=run_env,
        )
        assert process.stdout is not None
        for line in process.stdout:
            sys.stdout.write(line)
            log.write(line)
        return_code = process.wait()

    meta["return_code"] = return_code
    meta["finished_utc"] = datetime.now(timezone.utc).isoformat()
    meta["status"] = "ok" if return_code == 0 else "failed"
    trajectory_sources = materialize_orb_trajectory(
        output, args.settings.resolve()
    )
    meta["trajectory_source"] = trajectory_sources["primary"]
    meta["optimized_trajectory_source"] = trajectory_sources["optimized"]
    meta["keyframe_count"] = trajectory_sources["keyframe_count"]
    meta["frame_pose_count"] = trajectory_sources["frame_pose_count"]
    meta["keyframe_covariance_rank"] = trajectory_sources["covariance_rank"]
    meta["primary_trajectory_row_count"] = trajectory_row_count(
        output / "trajectory_est.tum"
    )
    meta["imu"] = imu_metadata(args.manifest, args.imu)
    (output / "run_meta.json").write_text(
        json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
