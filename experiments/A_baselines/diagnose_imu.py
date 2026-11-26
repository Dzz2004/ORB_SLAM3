#!/usr/bin/env python3
"""Check TartanAir gyro/accelerometer consistency against camera poses."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load_rows(path: Path) -> np.ndarray:
    return np.loadtxt(path, dtype=np.float64, ndmin=2)


def interp_positions(times: np.ndarray, positions: np.ndarray, query: np.ndarray) -> np.ndarray:
    return np.column_stack(
        [np.interp(query, times, positions[:, axis]) for axis in range(3)]
    )


def finite_diff(values: np.ndarray, times: np.ndarray) -> np.ndarray:
    return np.gradient(values, times, axis=0, edge_order=2)


def correlation_report(measured: np.ndarray, expected: np.ndarray) -> dict:
    measured_centered = measured - measured.mean(axis=0)
    expected_centered = expected - expected.mean(axis=0)
    denom = np.linalg.norm(measured_centered, axis=0) * np.linalg.norm(
        expected_centered, axis=0
    )
    correlations = np.divide(
        np.sum(measured_centered * expected_centered, axis=0),
        denom,
        out=np.zeros(3),
        where=denom > 0,
    )
    mapping, _, _, _ = np.linalg.lstsq(measured, expected, rcond=None)
    residual = expected - measured @ mapping
    return {
        "correlation_per_axis": correlations.tolist(),
        "mapping_measured_to_expected": mapping.tolist(),
        "mapping_rms": float(np.sqrt(np.mean(residual**2))),
        "measured_rms": float(np.sqrt(np.mean(measured**2))),
        "expected_rms": float(np.sqrt(np.mean(expected**2))),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()

    root = args.sequence_root.resolve()
    imu_dir = root / "imu"
    camera_times = load_rows(imu_dir / "cam_time.txt").reshape(-1)
    pose_rows = load_rows(root / "pose_lcam_front.txt")
    imu_times = load_rows(imu_dir / "imu_time.txt").reshape(-1)
    gyro = load_rows(imu_dir / "gyro.txt")
    acc = load_rows(imu_dir / "acc.txt")

    mask_camera = camera_times <= args.duration + 1e-12
    mask_imu = imu_times <= args.duration + 1e-12
    camera_times = camera_times[mask_camera]
    pose_rows = pose_rows[mask_camera]
    imu_times = imu_times[mask_imu]
    gyro = gyro[mask_imu]
    acc = acc[mask_imu]

    positions = pose_rows[:, :3]
    rotations = Rotation.from_quat(pose_rows[:, 3:7])
    slerp = Slerp(camera_times, rotations)
    rotations_at_imu = slerp(imu_times)
    positions_at_imu = interp_positions(camera_times, positions, imu_times)

    expected_gyro = np.empty((len(imu_times), 3), dtype=np.float64)
    for index in range(len(imu_times) - 1):
        dt = imu_times[index + 1] - imu_times[index]
        relative = rotations_at_imu[index].inv() * rotations_at_imu[index + 1]
        expected_gyro[index] = relative.as_rotvec() / dt
    expected_gyro[-1] = expected_gyro[-2]

    velocity = finite_diff(positions_at_imu, imu_times)
    acceleration_world = finite_diff(velocity, imu_times)
    acc_nominal = np.empty_like(acc)
    acc_up_gravity = np.empty_like(acc)
    gravity = np.array([0.0, 0.0, 9.81])
    for index, rotation in enumerate(rotations_at_imu):
        acc_nominal[index] = rotation.inv().apply(acceleration_world[index] - gravity)
        acc_up_gravity[index] = rotation.inv().apply(acceleration_world[index] + gravity)

    report = {
        "sequence_root": str(root),
        "duration_seconds": args.duration,
        "camera_samples": int(len(camera_times)),
        "imu_samples": int(len(imu_times)),
        "gyro_vs_pose": correlation_report(gyro, expected_gyro),
        "acc_vs_derivative_gravity_down": correlation_report(acc, acc_nominal),
        "acc_vs_derivative_gravity_up": correlation_report(acc, acc_up_gravity),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
