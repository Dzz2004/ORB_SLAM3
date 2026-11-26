#!/usr/bin/env python3
"""Evaluate one A baseline run with evo and write JSON plus a trajectory plot."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface


def read_tracking(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def tracking_summary(path: Path) -> dict[str, float | int]:
    rows = read_tracking(path)
    valid = [row["valid"] == "1" for row in rows]
    latency = [
        float(row["latency_ms"])
        for row in rows
        if row.get("latency_ms") not in (None, "", "NA")
    ]
    lost_events = 0
    seen_valid = False
    in_loss = False
    for is_valid in valid:
        if is_valid:
            seen_valid = True
            in_loss = False
        elif seen_valid and not in_loss:
            lost_events += 1
            in_loss = True
    return {
        "input_frames": len(rows),
        "valid_pose_frames": sum(valid),
        "valid_pose_ratio": sum(valid) / len(rows) if rows else 0.0,
        "lost_events": lost_events,
        "mean_latency_ms": float(np.mean(latency)) if latency else math.nan,
        "p95_latency_ms": float(np.percentile(latency, 95)) if latency else math.nan,
    }


def trajectory_row_count(path: Path) -> tuple[int, str | None]:
    if not path.exists():
        return 0, "estimated trajectory file does not exist"
    rows = 0
    try:
        with path.open(encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                fields = line.split()
                if not fields:
                    continue
                if len(fields) != 8:
                    return rows, (
                        f"line {line_number} has {len(fields)} fields; "
                        "expected 8"
                    )
                values = [float(value) for value in fields]
                if not np.isfinite(values).all():
                    return rows, f"line {line_number} contains non-finite values"
                rows += 1
    except (OSError, ValueError) as error:
        return rows, f"{type(error).__name__}: {error}"
    if rows == 0:
        return 0, "estimated trajectory file is empty"
    return rows, None


def set_statistics(
    target: dict[str, object],
    prefix: str,
    result: metrics.Metric,
) -> None:
    stats = result.get_result().stats
    for name in ("rmse", "mean", "median", "std"):
        target[f"{prefix}_{name}"] = float(stats[name])


def mark_unavailable(
    result: dict[str, object],
    status: str,
    error: str,
) -> None:
    result.update(
        {
            "status": status,
            "error": error,
            "ate_status": "unavailable",
            "orientation_ape_status": "unavailable",
            "rpe_translation_status": "unavailable",
            "rpe_rotation_status": "unavailable",
        }
    )


def largest_map_window(path: Path) -> dict[str, float | int] | None:
    rows = read_tracking(path)
    if not rows or "map_id" not in rows[0]:
        return None
    valid_rows = [
        row
        for row in rows
        if row.get("valid") == "1" and int(row.get("map_id", "-1")) >= 0
    ]
    if not valid_rows:
        return None
    counts = Counter(int(row["map_id"]) for row in valid_rows)
    map_id, count = counts.most_common(1)[0]
    map_rows = [row for row in valid_rows if int(row["map_id"]) == map_id]
    return {
        "map_id": map_id,
        "map_ids": sorted(counts),
        "valid_rows": count,
        "start": float(map_rows[0]["timestamp"]),
        "end": float(map_rows[-1]["timestamp"]),
    }


def slice_trajectory(traj: PoseTrajectory3D, start: float, end: float) -> PoseTrajectory3D:
    mask = (traj.timestamps >= start - 1e-9) & (traj.timestamps <= end + 1e-9)
    return PoseTrajectory3D(
        traj.positions_xyz[mask],
        traj.orientations_quat_wxyz[mask],
        traj.timestamps[mask],
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimated", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--tracking", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--alignment", required=True, choices=["sim3", "se3"]
    )
    parser.add_argument("--max-time-diff", default=0.02, type=float)
    parser.add_argument("--rpe-delta", default=1.0, type=float)
    parser.add_argument(
        "--rpe-unit", choices=["frames", "meters"], default="meters"
    )
    parser.add_argument("--plot", type=Path)
    parser.add_argument(
        "--map-segment", choices=["fixed", "largest", "all"], default="fixed"
    )
    args = parser.parse_args()

    result: dict[str, object] = {
        "alignment": args.alignment,
        "rpe_delta": args.rpe_delta,
        "rpe_delta_unit": args.rpe_unit,
    }
    try:
        result.update(tracking_summary(args.tracking))
        estimated_rows, estimated_file_error = trajectory_row_count(args.estimated)
        result["estimated_trajectory_rows"] = estimated_rows
        if estimated_file_error:
            mark_unavailable(
                result,
                "primary_trajectory_unavailable",
                estimated_file_error,
            )
            result["plot_written"] = False
        else:
            reference = file_interface.read_tum_trajectory_file(str(args.reference))
            estimated = file_interface.read_tum_trajectory_file(str(args.estimated))
            map_window = largest_map_window(args.tracking)
            result["map_ids_in_valid_tracking"] = (
                map_window["map_ids"] if map_window else []
            )
            if (
                map_window
                and args.map_segment == "fixed"
                and len(map_window["map_ids"]) > 1
            ):
                mark_unavailable(
                    result,
                    "invalid_map_discontinuity",
                    (
                        "Multiple maps are active in the fixed evaluation window; "
                        "ATE/RPE would mix incompatible coordinate frames."
                    ),
                )
                result["plot_written"] = False
            else:
                if map_window and args.map_segment == "largest":
                    reference = slice_trajectory(
                        reference, float(map_window["start"]), float(map_window["end"])
                    )
                    estimated = slice_trajectory(
                        estimated, float(map_window["start"]), float(map_window["end"])
                    )
                    result["evaluated_map_id"] = map_window["map_id"]
                    result["evaluated_map_valid_rows"] = map_window["valid_rows"]
                    result["evaluated_window_start"] = map_window["start"]
                    result["evaluated_window_end"] = map_window["end"]
                else:
                    result["evaluated_window_start"] = float(reference.timestamps[0])
                    result["evaluated_window_end"] = float(reference.timestamps[-1])

                reference = slice_trajectory(
                    reference,
                    float(result["evaluated_window_start"]),
                    float(result["evaluated_window_end"]),
                )
                estimated = slice_trajectory(
                    estimated,
                    float(result["evaluated_window_start"]),
                    float(result["evaluated_window_end"]),
                )
                reference, estimated = sync.associate_trajectories(
                    reference, estimated, max_diff=args.max_time_diff
                )
                result["matched_pose_frames"] = len(reference.timestamps)
                if len(reference.timestamps) < 2:
                    mark_unavailable(
                        result,
                        "insufficient_associated_poses",
                        (
                            "Fewer than two reference and estimated poses have "
                            f"timestamps within {args.max_time_diff:.3f} s."
                        ),
                    )
                    result["plot_written"] = False
                else:
                    estimated.align(
                        reference,
                        correct_scale=args.alignment == "sim3",
                        correct_only_scale=False,
                    )
                    result["status"] = "ok"
                    result.pop("error", None)

                    try:
                        ape = metrics.APE(metrics.PoseRelation.translation_part)
                        ape.process_data((reference, estimated))
                        set_statistics(result, "ate", ape)
                        result["ate_status"] = "ok"
                    except Exception as error:
                        result["ate_status"] = "unavailable"
                        result["ate_error"] = f"{type(error).__name__}: {error}"

                    try:
                        orientation_ape = metrics.APE(
                            metrics.PoseRelation.rotation_angle_deg
                        )
                        orientation_ape.process_data((reference, estimated))
                        orientation_stats = orientation_ape.get_result().stats
                        result["orientation_ape_rmse_deg"] = float(
                            orientation_stats["rmse"]
                        )
                        result["orientation_ape_status"] = "ok"
                    except Exception as error:
                        result["orientation_ape_status"] = "unavailable"
                        result["orientation_ape_error"] = (
                            f"{type(error).__name__}: {error}"
                        )

                    unit = (
                        metrics.Unit.meters
                        if args.rpe_unit == "meters"
                        else metrics.Unit.frames
                    )
                    for relation, metric_prefix, status_key in (
                        (
                            metrics.PoseRelation.translation_part,
                            "rpe_translation",
                            "rpe_translation_status",
                        ),
                        (
                            metrics.PoseRelation.rotation_angle_deg,
                            "rpe_rotation_deg",
                            "rpe_rotation_status",
                        ),
                    ):
                        try:
                            rpe = metrics.RPE(relation, args.rpe_delta, unit)
                            rpe.process_data((reference, estimated))
                            set_statistics(result, metric_prefix, rpe)
                            result[status_key] = "ok"
                        except Exception as error:
                            result[status_key] = "unavailable"
                            result[f"{metric_prefix}_error"] = (
                                f"{type(error).__name__}: {error}"
                            )

                    if result["ate_status"] != "ok":
                        result["status"] = "ate_unavailable"

                    if args.plot and result["ate_status"] == "ok":
                        positions_ref = reference.positions_xyz
                        positions_est = estimated.positions_xyz
                        fig = plt.figure(figsize=(7, 6))
                        axis = fig.add_subplot(111)
                        axis.plot(
                            positions_ref[:, 0],
                            positions_ref[:, 1],
                            label="Ground truth",
                        )
                        axis.plot(
                            positions_est[:, 0],
                            positions_est[:, 1],
                            label="Estimated",
                        )
                        axis.set_aspect("equal", adjustable="datalim")
                        axis.set_xlabel("x [m]")
                        axis.set_ylabel("y [m]")
                        axis.set_title(
                            f"TartanAir trajectory ({args.alignment} alignment)"
                        )
                        axis.legend()
                        axis.grid(True, alpha=0.3)
                        fig.tight_layout()
                        args.plot.parent.mkdir(parents=True, exist_ok=True)
                        fig.savefig(args.plot, dpi=160)
                        plt.close(fig)
                        result["plot_written"] = True
                    else:
                        result["plot_written"] = False
    except Exception as error:
        result.update({"status": "failed", "error": f"{type(error).__name__}: {error}"})
        result.setdefault("ate_status", "unavailable")
        result.setdefault("rpe_translation_status", "unavailable")
        result.setdefault("rpe_rotation_status", "unavailable")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return (
        0
        if result.get("status")
        in {
            "ok",
            "ate_unavailable",
            "invalid_map_discontinuity",
            "primary_trajectory_unavailable",
            "insufficient_associated_poses",
        }
        else 2
    )


if __name__ == "__main__":
    raise SystemExit(main())
