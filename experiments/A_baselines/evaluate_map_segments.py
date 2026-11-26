#!/usr/bin/env python3
"""Evaluate contiguous single-map hard segments without concatenating maps."""

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
MIN_SEGMENT_FRAMES = 8
RPE_DELTA_FRAMES = 5


def read_tracking(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def map_segments(rows: list[dict[str, str]]) -> list[list[dict[str, str]]]:
    segments = []
    current = []
    current_map = None
    for row in rows:
        valid = row.get("valid") == "1"
        map_id = int(row.get("map_id", "-1"))
        if valid and map_id >= 0 and (
            current_map is None or map_id == current_map
        ):
            current.append(row)
            current_map = map_id
        else:
            if current:
                segments.append(current)
            current = [row] if valid and map_id >= 0 else []
            current_map = map_id if current else None
    if current:
        segments.append(current)
    return segments


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
    reference_full = file_interface.read_tum_trajectory_file(
        str(input_root / "trajectory_gt.tum")
    )

    rows = []
    for mode in MODES:
        for seed in SEEDS:
            run_dir = run_root / f"{mode}_seed{seed}"
            tracking = read_tracking(run_dir / "tracking.csv")
            estimated_full = file_interface.read_tum_trajectory_file(
                str(run_dir / "trajectory_frames.tum")
            )
            segments = map_segments(tracking)
            for segment_index, segment in enumerate(segments):
                map_id = int(segment[0]["map_id"])
                start = float(segment[0]["timestamp"])
                end = float(segment[-1]["timestamp"])
                result = {
                    "difficulty": "hard",
                    "trajectory": "P001",
                    "mode": mode,
                    "seed": seed,
                    "segment_index": segment_index,
                    "map_id": map_id,
                    "segment_start_s": start,
                    "segment_end_s": end,
                    "tracking_frames": len(segment),
                }
                if len(segment) < MIN_SEGMENT_FRAMES:
                    result["status"] = "too_short"
                    rows.append(result)
                    continue
                try:
                    reference = slice_trajectory(reference_full, start, end)
                    estimated = slice_trajectory(estimated_full, start, end)
                    matched_reference, matched_estimated = (
                        sync.associate_trajectories(
                            reference, estimated, max_diff=0.02
                        )
                    )
                    if len(matched_reference.timestamps) < 6:
                        raise ValueError(
                            "fewer than six associated poses"
                        )
                    matched_estimated.align(
                        matched_reference,
                        correct_scale=mode == "rgb",
                        correct_only_scale=False,
                    )
                    ape = metrics.APE(metrics.PoseRelation.translation_part)
                    ape.process_data((matched_reference, matched_estimated))
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
                            "matched_pose_frames": len(
                                matched_reference.timestamps
                            ),
                            "ate_rmse_m": float(
                                ape.get_result().stats["rmse"]
                            ),
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

    fieldnames = []
    for row in rows:
        for field in row:
            if field not in fieldnames:
                fieldnames.append(field)
    metrics_csv = result_root / "hard_map_segments.csv"
    with metrics_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=fieldnames, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)
    (result_root / "hard_map_segments.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    summary = []
    for mode in MODES:
        selected = [row for row in rows if row["mode"] == mode]
        valid = [row for row in selected if row["status"] == "ok"]
        summary.append(
            {
                "difficulty": "hard",
                "trajectory": "P001",
                "mode": mode,
                "segment_count": len(selected),
                "evaluable_segment_count": len(valid),
                "evaluable_seed_count": len({row["seed"] for row in valid}),
                "ate_rmse_m_median": (
                    float(np.median([row["ate_rmse_m"] for row in valid]))
                    if valid
                    else None
                ),
                "rpe_translation_rmse_m_median": (
                    float(
                        np.median(
                            [row["rpe_translation_rmse_m"] for row in valid]
                        )
                    )
                    if valid
                    else None
                ),
                "rpe_rotation_deg_rmse_median": (
                    float(
                        np.median(
                            [row["rpe_rotation_deg_rmse"] for row in valid]
                        )
                    )
                    if valid
                    else None
                ),
            }
        )
    summary_csv = result_root / "hard_map_segment_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(summary[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(summary)
    (result_root / "hard_map_segment_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    figure, axes = plt.subplots(1, 2, figsize=(13, 5))
    for mode_index, mode in enumerate(MODES):
        selected = [
            row
            for row in rows
            if row["mode"] == mode and row["status"] == "ok"
        ]
        axes[0].scatter(
            [row["tracking_frames"] for row in selected],
            [row["ate_rmse_m"] for row in selected],
            label=mode,
            color=("#4C78A8", "#F58518", "#54A24B")[mode_index],
        )
        axes[1].scatter(
            [row["tracking_frames"] for row in selected],
            [row["rpe_translation_rmse_m"] for row in selected],
            label=mode,
            color=("#4C78A8", "#F58518", "#54A24B")[mode_index],
        )
    axes[0].set_xlabel("Tracked frames in segment")
    axes[0].set_ylabel("Segment ATE RMSE [m]")
    axes[1].set_xlabel("Tracked frames in segment")
    axes[1].set_ylabel("Segment RPE translation RMSE [m]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    axes[0].legend()
    figure.suptitle("Hard P001: per-map segment accuracy")
    figure.tight_layout()
    figure.savefig(result_root / "figures/hard_map_segments.png", dpi=170)
    plt.close(figure)
    print(metrics_csv)
    print(summary_csv)


if __name__ == "__main__":
    main()
