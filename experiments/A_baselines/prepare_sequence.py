#!/usr/bin/env python3
"""Prepare a TartanAir trajectory for the ORB-SLAM3 baseline runner."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import cv2
import numpy as np


FRAME_ID_RE = re.compile(r"^(\d+)_")


def read_vector(path: Path) -> np.ndarray:
    values = np.loadtxt(path, dtype=np.float64, ndmin=2)
    if values.ndim != 2:
        raise ValueError(f"Unexpected array shape in {path}: {values.shape}")
    return values


def frame_id(path: Path) -> int:
    match = FRAME_ID_RE.match(path.name)
    if not match:
        raise ValueError(f"Cannot parse frame id from {path.name}")
    return int(match.group(1))


def decode_depth(path: Path) -> np.ndarray:
    encoded = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if encoded is None:
        raise ValueError(f"Cannot read depth image: {path}")
    if encoded.ndim == 2 and encoded.dtype == np.float32:
        return encoded
    if encoded.dtype == np.uint8 and encoded.ndim == 3 and encoded.shape[2] == 4:
        return np.squeeze(encoded.view("<f4"), axis=-1)
    raise ValueError(
        f"Unsupported depth encoding in {path}: "
        f"shape={encoded.shape}, dtype={encoded.dtype}"
    )


def ensure_same_length(name: str, values: list[Path] | np.ndarray, expected: int) -> None:
    actual = len(values)
    if actual != expected:
        raise ValueError(f"{name} has {actual} rows, expected {expected}")


def write_manifest(
    output: Path,
    timestamps: np.ndarray,
    rgb_paths: list[Path],
    depth_by_id: dict[int, Path],
    pose_rows: np.ndarray,
) -> None:
    with (output / "manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp", "rgb_path", "depth_path", "gt_index", "frame_id"])
        for index, (timestamp, rgb_path) in enumerate(zip(timestamps, rgb_paths)):
            fid = frame_id(rgb_path)
            depth_path = depth_by_id.get(fid)
            if depth_path is None:
                raise ValueError(f"No depth image found for RGB frame {fid}")
            writer.writerow(
                [
                    f"{float(timestamp):.17g}",
                    rgb_path.resolve(),
                    depth_path.resolve(),
                    index,
                    fid,
                ]
            )


def write_imu(output: Path, imu_dir: Path) -> int:
    times = read_vector(imu_dir / "imu_time.txt").reshape(-1)
    acc = read_vector(imu_dir / "acc.txt")
    gyro = read_vector(imu_dir / "gyro.txt")
    ensure_same_length("acc.txt", acc, len(times))
    ensure_same_length("gyro.txt", gyro, len(times))
    if not (acc.shape[1] == 3 and gyro.shape[1] == 3):
        raise ValueError("TartanAir acc/gyro files must each have three columns")

    with (output / "imu.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp", "gx", "gy", "gz", "ax", "ay", "az"])
        for stamp, angular, acceleration in zip(times, gyro, acc):
            writer.writerow(
                [
                    f"{float(stamp):.17g}",
                    f"{float(angular[0]):.17g}",
                    f"{float(angular[1]):.17g}",
                    f"{float(angular[2]):.17g}",
                    f"{float(acceleration[0]):.17g}",
                    f"{float(acceleration[1]):.17g}",
                    f"{float(acceleration[2]):.17g}",
                ]
            )
    return len(times)


def write_ground_truth(output: Path, timestamps: np.ndarray, pose_rows: np.ndarray) -> None:
    if pose_rows.shape[1] != 7:
        raise ValueError("TartanAir pose must contain translation and quaternion columns")
    with (output / "trajectory_gt.tum").open("w", encoding="utf-8") as handle:
        for stamp, row in zip(timestamps, pose_rows):
            handle.write(
                f"{float(stamp):.17g} "
                f"{row[0]:.17g} {row[1]:.17g} {row[2]:.17g} "
                f"{row[3]:.17g} {row[4]:.17g} {row[5]:.17g} {row[6]:.17g}\n"
            )


def validate_depth(depth_paths: list[Path]) -> dict[str, float]:
    sample_indices = sorted({0, len(depth_paths) // 2, len(depth_paths) - 1})
    ratios: list[float] = []
    minima: list[float] = []
    maxima: list[float] = []
    for index in sample_indices:
        depth = decode_depth(depth_paths[index])
        finite = np.isfinite(depth)
        plausible = finite & (depth > 0.0) & (depth < 1000.0)
        ratios.append(float(plausible.mean()))
        minima.append(float(np.min(depth[plausible])) if plausible.any() else 0.0)
        maxima.append(float(np.max(depth[plausible])) if plausible.any() else 0.0)
    return {
        "sampled_frames": float(len(sample_indices)),
        "min_valid_ratio": min(ratios),
        "max_valid_ratio": max(ratios),
        "min_plausible_depth_m": min(minima),
        "max_plausible_depth_m": max(maxima),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence-root", required=True, type=Path)
    parser.add_argument("--rgb-subdir", default="image_lcam_front")
    parser.add_argument("--depth-subdir", default="depth_lcam_front")
    parser.add_argument("--pose-file", default="pose_lcam_front.txt")
    parser.add_argument("--imu-subdir", default="imu")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--num-frames", type=int, default=0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    root = args.sequence_root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    rgb_dir = root / args.rgb_subdir
    depth_dir = root / args.depth_subdir
    imu_dir = root / args.imu_subdir
    rgb_paths = sorted(rgb_dir.glob("*.png"), key=frame_id)
    depth_paths = sorted(depth_dir.glob("*.png"), key=frame_id)
    if not rgb_paths:
        raise ValueError(f"No RGB images found in {rgb_dir}")
    if not depth_paths:
        raise ValueError(f"No depth images found in {depth_dir}")

    timestamps = read_vector(imu_dir / "cam_time.txt").reshape(-1)
    pose_rows = read_vector(root / args.pose_file)
    ensure_same_length("RGB list", rgb_paths, len(timestamps))
    ensure_same_length("pose file", pose_rows, len(timestamps))

    start = max(0, args.start_frame)
    if start >= len(rgb_paths):
        raise ValueError(f"Start frame {start} is outside {len(rgb_paths)} frames")
    if start > 0:
        rgb_paths = rgb_paths[start:]
        timestamps = timestamps[start:]
        pose_rows = pose_rows[start:]

    if args.num_frames > 0:
        if args.num_frames > len(rgb_paths):
            raise ValueError(
                f"Requested {args.num_frames} frames but only {len(rgb_paths)} exist"
            )
        rgb_paths = rgb_paths[: args.num_frames]
        timestamps = timestamps[: args.num_frames]
        pose_rows = pose_rows[: args.num_frames]

    depth_by_id = {frame_id(path): path for path in depth_paths}
    write_manifest(output, timestamps, rgb_paths, depth_by_id, pose_rows)
    imu_count = write_imu(output, imu_dir)
    write_ground_truth(output, timestamps, pose_rows)
    depth_stats = validate_depth([depth_by_id[frame_id(path)] for path in rgb_paths])

    meta = {
        "sequence_root": str(root),
        "rgb_subdir": args.rgb_subdir,
        "depth_subdir": args.depth_subdir,
        "frame_count": len(rgb_paths),
        "imu_count": imu_count,
        "pose_count": len(pose_rows),
        "start_frame": start,
        "requested_frame_count": args.num_frames,
        "timestamp_start": float(timestamps[0]),
        "timestamp_end": float(timestamps[-1]),
        "depth": depth_stats,
        "sensor": {
            "camera": "Pinhole 640x640 fx=fy=320 cx=cy=320 distortion=0",
            "camera_hz": 10.0,
            "imu_hz": 100.0,
            "imu_generator": "TartanAir synthetic IMU aligned with lcam_front",
        },
    }
    (output / "input_validation.json").write_text(
        json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(meta, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
