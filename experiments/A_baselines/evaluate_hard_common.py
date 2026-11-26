#!/usr/bin/env python3
"""Evaluate the selected hard interval with one trajectory source for every mode."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface


MODES = ("rgb", "rgb_imu", "rgbd_imu")
SEEDS = (0, 1, 2)
START = 12.9
END = 14.9
RPE_DELTA_FRAMES = 5


def slice_trajectory(
    trajectory: PoseTrajectory3D, start: float, end: float
) -> PoseTrajectory3D:
    mask = (
        (trajectory.timestamps >= start - 1e-6)
        & (trajectory.timestamps <= end + 1e-6)
    )
    return PoseTrajectory3D(
        trajectory.positions_xyz[mask],
        trajectory.orientations_quat_wxyz[mask],
        trajectory.timestamps[mask],
    )


def inspect_tracking(path: Path) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    window = [
        row
        for row in rows
        if START - 1e-6 <= float(row["timestamp"]) <= END + 1e-6
    ]
    valid = [row for row in window if row["valid"] == "1"]
    maps = sorted({int(row["map_id"]) for row in valid if int(row["map_id"]) >= 0})
    return {
        "window_frames": len(window),
        "valid_window_frames": len(valid),
        "valid_pose_ratio": len(valid) / len(window) if window else 0.0,
        "lost_window_frames": len(window) - len(valid),
        "map_ids": maps,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    input_root = (
        root / "results/A/inputs_final/OldTownSummer/hard/P001_start100"
    )
    run_root = root / "results/A/baseline_final/hard/P001_window_start100"
    result_root = root / "results/A"
    figure_root = result_root / "figures"
    figure_root.mkdir(parents=True, exist_ok=True)

    reference = file_interface.read_tum_trajectory_file(
        str(input_root / "trajectory_gt.tum")
    )
    reference = slice_trajectory(reference, START, END)
    rows = []
    for mode in MODES:
        for seed in SEEDS:
            run_dir = run_root / f"{mode}_seed{seed}"
            tracking = inspect_tracking(run_dir / "tracking.csv")
            result = {
                "dataset": "TartanAirV2",
                "environment": "OldTownSummer",
                "trajectory": "P001",
                "difficulty": "hard",
                "mode": mode,
                "seed": seed,
                "evaluation_type": "common_window_dense_body_trajectory",
                "window_start_s": START,
                "window_end_s": END,
                **tracking,
            }
            try:
                if tracking["map_ids"] != [0]:
                    raise ValueError(
                        f"common window is not single-map 0: {tracking['map_ids']}"
                    )
                estimated = file_interface.read_tum_trajectory_file(
                    str(run_dir / "trajectory_frames.tum")
                )
                estimated = slice_trajectory(estimated, START, END)
                matched_reference, matched_estimated = (
                    sync.associate_trajectories(
                        reference, estimated, max_diff=0.02
                    )
                )
                if len(matched_reference.timestamps) < 3:
                    raise ValueError(
                        f"too few matched frames: {len(matched_reference.timestamps)}"
                    )
                matched_estimated.align(
                    matched_reference,
                    correct_scale=mode == "rgb",
                    correct_only_scale=False,
                )
                ape = metrics.APE(metrics.PoseRelation.translation_part)
                ape.process_data((matched_reference, matched_estimated))
                orientation = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
                orientation.process_data((matched_reference, matched_estimated))
                rpe_t = metrics.RPE(
                    metrics.PoseRelation.translation_part,
                    RPE_DELTA_FRAMES,
                    metrics.Unit.frames,
                )
                rpe_t.process_data((matched_reference, matched_estimated))
                rpe_r = metrics.RPE(
                    metrics.PoseRelation.rotation_angle_deg,
                    RPE_DELTA_FRAMES,
                    metrics.Unit.frames,
                )
                rpe_r.process_data((matched_reference, matched_estimated))
                result.update(
                    {
                        "status": "ok",
                        "matched_pose_frames": len(matched_reference.timestamps),
                        "ate_rmse_m": float(ape.get_result().stats["rmse"]),
                        "orientation_ape_rmse_deg": float(
                            orientation.get_result().stats["rmse"]
                        ),
                        "rpe_delta_frames": RPE_DELTA_FRAMES,
                        "rpe_translation_rmse_m": float(
                            rpe_t.get_result().stats["rmse"]
                        ),
                        "rpe_rotation_deg_rmse": float(
                            rpe_r.get_result().stats["rmse"]
                        ),
                    }
                )
            except Exception as error:
                result.update(
                    {
                        "status": "failed",
                        "error": f"{type(error).__name__}: {error}",
                    }
                )

            rows.append(result)
            (run_dir / "metrics_common_hard.json").write_text(
                json.dumps(result, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            print(mode, seed, result["status"], result.get("ate_rmse_m"))

    metrics_csv = result_root / "hard_common_metrics.csv"
    metric_fields = []
    for row in rows:
        for field in row:
            if field not in metric_fields:
                metric_fields.append(field)
    with metrics_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=metric_fields, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)
    (result_root / "hard_common_metrics.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    aggregate = []
    for mode in MODES:
        selected = [row for row in rows if row["mode"] == mode]
        valid = [row for row in selected if row["status"] == "ok"]
        item = {
            "difficulty": "hard",
            "trajectory": "P001",
            "mode": mode,
            "seed_count": len(selected),
            "valid_seed_count": len(valid),
        }
        for field in (
            "ate_rmse_m",
            "rpe_translation_rmse_m",
            "rpe_rotation_deg_rmse",
            "orientation_ape_rmse_deg",
            "valid_pose_ratio",
            "lost_window_frames",
        ):
            values = np.asarray([row[field] for row in valid], dtype=float)
            item[f"{field}_mean"] = float(values.mean()) if len(values) else None
            item[f"{field}_std"] = (
                float(values.std(ddof=1)) if len(values) > 1 else 0.0
            )
        aggregate.append(item)

    aggregate_csv = result_root / "hard_common_aggregate.csv"
    with aggregate_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(aggregate[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(aggregate)
    (result_root / "hard_common_aggregate.json").write_text(
        json.dumps(aggregate, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    figure, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    x = np.arange(len(MODES))
    panels = (
        ("ate_rmse_m", "ATE RMSE [m]"),
        ("rpe_translation_rmse_m", "RPE translation [m]"),
        ("rpe_rotation_deg_rmse", "RPE rotation [deg]"),
    )
    for axis, (field, label) in zip(axes, panels, strict=True):
        means = np.asarray(
            [
                aggregate_item[f"{field}_mean"]
                if aggregate_item[f"{field}_mean"] is not None
                else np.nan
                for aggregate_item in aggregate
            ],
            dtype=float,
        )
        stds = np.asarray(
            [
                aggregate_item[f"{field}_std"]
                if aggregate_item[f"{field}_std"] is not None
                else 0.0
                for aggregate_item in aggregate
            ],
            dtype=float,
        )
        errors = np.vstack(
            [
                np.where(np.isnan(means), 0.0, np.minimum(means, stds)),
                stds,
            ]
        )
        axis.bar(
            x,
            means,
            yerr=errors,
            capsize=4,
            color=("#4C78A8", "#F58518", "#54A24B"),
        )
        axis.set_xticks(x, MODES)
        axis.set_ylabel(label)
        axis.grid(True, axis="y", alpha=0.3)
    figure.suptitle("Hard P001 conditional accuracy: common 12.9-14.9 s")
    figure.tight_layout()
    figure.savefig(figure_root / "hard_common_summary.png", dpi=170)
    plt.close(figure)

    for mode in MODES:
        run_dir = run_root / f"{mode}_seed0"
        estimated = file_interface.read_tum_trajectory_file(
            str(run_dir / "trajectory_frames.tum")
        )
        estimated = slice_trajectory(estimated, START, END)
        matched_reference, matched_estimated = sync.associate_trajectories(
            reference, estimated, max_diff=0.02
        )
        matched_estimated.align(
            matched_reference,
            correct_scale=mode == "rgb",
            correct_only_scale=False,
        )
        figure, axis = plt.subplots(figsize=(7, 6))
        axis.plot(
            matched_reference.positions_xyz[:, 0],
            matched_reference.positions_xyz[:, 1],
            label="Ground truth",
        )
        axis.plot(
            matched_estimated.positions_xyz[:, 0],
            matched_estimated.positions_xyz[:, 1],
            label="Estimated",
        )
        axis.set_aspect("equal", adjustable="datalim")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_title(f"Hard P001 common window: {mode}, seed 0")
        axis.legend()
        axis.grid(True, alpha=0.3)
        figure.tight_layout()
        figure.savefig(figure_root / f"hard_{mode}_overlay.png", dpi=160)
        plt.close(figure)
    print(metrics_csv)
    print(aggregate_csv)


if __name__ == "__main__":
    main()
