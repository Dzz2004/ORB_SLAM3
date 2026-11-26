#!/usr/bin/env python3
"""Summarize all six available TartanAir trajectory directories."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
import yaml


MODES = ("rgb", "rgb_imu", "rgbd_imu")
SEEDS = (0, 1, 2)


def trajectory_row_count(path: Path) -> int:
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(encoding="utf-8") as handle:
        return sum(1 for line in handle if line.strip())


def derived_trajectory_status(
    metadata: dict[str, object], run_dir: Path
) -> tuple[str, int, int, int | None]:
    keyframe_path = run_dir / "keyframe_trajectory.tum"
    frame_path = run_dir / "trajectory_frames.tum"
    primary_path = run_dir / "trajectory_est.tum"
    keyframe_count = int(
        metadata.get("keyframe_count", trajectory_row_count(keyframe_path))
    )
    frame_pose_count = int(
        metadata.get("frame_pose_count", trajectory_row_count(frame_path))
    )
    primary_count = trajectory_row_count(primary_path)
    covariance_rank = metadata.get("keyframe_covariance_rank")
    if primary_count > 0:
        source = "optimized_keyframe_body"
    elif keyframe_count == 0:
        source = "unavailable_empty_keyframe"
    elif keyframe_count < 3:
        source = "unavailable_sparse_keyframe"
    else:
        source = "unavailable_degenerate_keyframe"
    if covariance_rank is not None:
        covariance_rank = int(covariance_rank)
    return source, keyframe_count, frame_pose_count, covariance_rank


def format_metric(item: dict[str, object], field: str) -> str:
    mean = item.get(f"{field}_mean")
    std = item.get(f"{field}_std")
    if mean is None:
        return "N/A"
    return f"{float(mean):.3f} ± {float(std or 0.0):.3f}"


def render_report(
    aggregate: list[dict[str, object]],
    rows: list[dict[str, object]],
) -> str:
    status_counts = Counter(str(row["status"]) for row in rows)
    tracking_available = sum(
        row["tracking_pose_status"] == "available" for row in rows
    )
    lines = [
        "# A Baseline: All Available Trajectory Sequences",
        "",
        "## Scope",
        "",
        "The local `OldTownSummer` TartanAir V2 package contains three Easy and "
        "three Hard trajectory directories. This matrix uses all six directories "
        "with RGB, RGB+IMU and RGB-D+IMU for seeds 0, 1 and 2.",
        "",
        "This produces `6 trajectories x 3 modes x 3 seeds = 54` runs.",
        "",
        "## Evaluation rules",
        "",
        "- Use the optimized-keyframe body trajectory as the primary comparison source.",
        "- RGB uses Sim(3); inertial modes use SE(3).",
        "- Full-window ATE/RPE require a single map and a non-empty primary trajectory.",
        "- ATE and RPE are evaluated independently; RPE unavailability does not erase ATE.",
        "- Coverage, loss events and latency always use all 100 tracking frames.",
        "- Per-frame by-map plots are diagnostics, never substitutes for the primary "
        "continuous trajectory.",
        "",
        "## Status accounting",
        "",
        f"- Successful continuous primary evaluation: `{status_counts['ok']}` runs.",
        f"- No usable primary keyframe trajectory: "
        f"`{status_counts['primary_trajectory_unavailable']}` runs.",
        f"- Primary trajectory exists but the window crosses maps: "
        f"`{status_counts['invalid_map_discontinuity']}` runs.",
        f"- Tracking poses were available in `{tracking_available}` of "
        f"`{len(rows)}` runs, including failed primary-trajectory runs.",
        "",
        "`primary_trajectory_unavailable` is not the same as zero tracking frames. "
        "It means ORB-SLAM3 produced no usable optimized keyframe trajectory "
        "(for example zero keyframes), while per-frame diagnostic poses may still exist.",
    ]
    for difficulty in ("easy", "hard"):
        lines.extend(
            [
                "",
                f"## {difficulty.title()} results",
                "",
                "Means use only seeds with a valid ATE or RPE. Coverage and loss "
                "events use all three seeds.",
                "",
                "| Trajectory | Mode | Primary | ATE seeds | RPE-T seeds | "
                "RPE-R seeds | ATE [m] | RPE-T [m] | RPE-R [deg] | "
                "Coverage | Loss events |",
                "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for item in [
            value
            for value in aggregate
            if value["difficulty"] == difficulty
        ]:
            lines.append(
                "| {trajectory} | {mode} | {primary}/3 | {ate}/3 | "
                "{rpe_t}/3 | {rpe_r}/3 | {ate_metric} | {rpe_t_metric} | "
                "{rpe_r_metric} | {coverage:.1f}% | {loss:.1f} |".format(
                    trajectory=item["trajectory"],
                    mode=item["mode"],
                    primary=item["primary_trajectory_available_count"],
                    ate=item["ate_available_count"],
                    rpe_t=item["rpe_translation_available_count"],
                    rpe_r=item["rpe_rotation_available_count"],
                    ate_metric=format_metric(item, "ate_rmse_m"),
                    rpe_t_metric=format_metric(
                        item, "rpe_translation_rmse_m"
                    ),
                    rpe_r_metric=format_metric(
                        item, "rpe_rotation_deg_rmse"
                    ),
                    coverage=100.0 * float(item["valid_pose_ratio_mean"]),
                    loss=float(item["lost_events_mean"]),
                )
            )
    lines.extend(
        [
            "",
            "## Output files",
            "",
            "```text",
            "results/A/all_sequences_metrics.csv",
            "results/A/all_sequences_metrics.json",
            "results/A/all_sequences_aggregate.csv",
            "results/A/all_sequences_aggregate.json",
            "results/A/available_sequence_trajectory_manifest.csv",
            "results/A/available_sequences/trajectories/",
            "results/A/figures/easy_available_trajectories.png",
            "results/A/figures/hard_available_trajectories.png",
            "```",
            "",
            "## Limitations",
            "",
            "- The package contains three physical trajectories per difficulty, "
            "not five.",
            "- Windows are fixed 100-frame samples, not complete sequence lengths.",
            "- Hard runs frequently reset maps or fail inertial initialization.",
            "- Easy P001 RGB+IMU ATE is available in all three runs, but 1 m RPE "
            "pairs are unavailable because the exported keyframe trajectory is too sparse.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument(
        "--matrix",
        type=Path,
        help="Sequence matrix; defaults to experiments/A_baselines/available_sequences.yaml.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        help="Directory for summary files; defaults to ROOT/results/A.",
    )
    args = parser.parse_args()
    root = args.root.resolve()
    matrix_path = (
        args.matrix.resolve()
        if args.matrix
        else root / "experiments/A_baselines/available_sequences.yaml"
    )
    matrix = yaml.safe_load(
        matrix_path.read_text(encoding="utf-8")
    )
    summary_root = (
        args.output_root.resolve()
        if args.output_root
        else root / "results/A"
    )
    summary_root.mkdir(parents=True, exist_ok=True)
    rows = []
    for sequence in matrix["sequences"]:
        result_root = root / sequence["result_root"]
        for mode in MODES:
            for seed in SEEDS:
                run_dir = result_root / f"{mode}_seed{seed}"
                metrics = json.loads(
                    (run_dir / "metrics.json").read_text(encoding="utf-8")
                )
                metadata_path = run_dir / "run_meta.json"
                metadata = (
                    json.loads(metadata_path.read_text(encoding="utf-8"))
                    if metadata_path.exists()
                    else {}
                )
                (
                    derived_source,
                    keyframe_count,
                    frame_pose_count,
                    covariance_rank,
                ) = derived_trajectory_status(metadata, run_dir)
                primary_count = trajectory_row_count(
                    run_dir / "trajectory_est.tum"
                )
                valid_pose_frames = metrics.get("valid_pose_frames")
                rows.append(
                    {
                        "sample_id": sequence["id"],
                        "difficulty": sequence["difficulty"],
                        "trajectory": sequence["trajectory"],
                        "start_frame": sequence["start_frame"],
                        "mode": mode,
                        "seed": seed,
                        "status": metrics.get("status"),
                        "tracking_pose_status": (
                            "available"
                            if valid_pose_frames is not None
                            and int(valid_pose_frames) > 0
                            else "unavailable"
                        ),
                        "primary_trajectory_status": (
                            "available"
                            if primary_count > 0
                            else "unavailable"
                        ),
                        "trajectory_source": derived_source,
                        "keyframe_count": keyframe_count,
                        "frame_pose_count": frame_pose_count,
                        "keyframe_covariance_rank": covariance_rank,
                        "ate_status": metrics.get("ate_status"),
                        "rpe_translation_status": metrics.get(
                            "rpe_translation_status"
                        ),
                        "rpe_rotation_status": metrics.get("rpe_rotation_status"),
                        "map_ids": ",".join(
                            str(value)
                            for value in metrics.get(
                                "map_ids_in_valid_tracking", []
                            )
                        ),
                        "ate_rmse_m": metrics.get("ate_rmse"),
                        "rpe_translation_rmse_m": metrics.get(
                            "rpe_translation_rmse"
                        ),
                        "rpe_rotation_deg_rmse": metrics.get(
                            "rpe_rotation_deg_rmse"
                        ),
                        "valid_pose_ratio": metrics.get("valid_pose_ratio"),
                        "valid_pose_frames": metrics.get("valid_pose_frames"),
                        "lost_events": metrics.get("lost_events"),
                        "mean_latency_ms": metrics.get("mean_latency_ms"),
                        "p95_latency_ms": metrics.get("p95_latency_ms"),
                    }
                )

    fieldnames = []
    for row in rows:
        for field in row:
            if field not in fieldnames:
                fieldnames.append(field)
    with (summary_root / "all_sequences_metrics.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(
            handle, fieldnames=fieldnames, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)
    (summary_root / "all_sequences_metrics.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    aggregate = []
    for sequence in matrix["sequences"]:
        for mode in MODES:
            selected = [
                row
                for row in rows
                if row["sample_id"] == sequence["id"] and row["mode"] == mode
            ]
            valid = [row for row in selected if row["status"] == "ok"]
            item = {
                "sample_id": sequence["id"],
                "difficulty": sequence["difficulty"],
                "trajectory": sequence["trajectory"],
                "start_frame": sequence["start_frame"],
                "mode": mode,
                "run_count": len(selected),
                "ok_count": len(valid),
                "tracking_pose_available_count": sum(
                    row["tracking_pose_status"] == "available"
                    for row in selected
                ),
                "primary_trajectory_available_count": sum(
                    row["primary_trajectory_status"] == "available"
                    for row in selected
                ),
                "ate_available_count": sum(
                    row["ate_status"] == "ok" for row in selected
                ),
                "rpe_translation_available_count": sum(
                    row["rpe_translation_status"] == "ok" for row in selected
                ),
                "rpe_rotation_available_count": sum(
                    row["rpe_rotation_status"] == "ok" for row in selected
                ),
                "status_counts": json.dumps(
                    {
                        status: sum(row["status"] == status for row in selected)
                        for status in sorted(
                            {str(row["status"]) for row in selected}
                        )
                    },
                    sort_keys=True,
                ),
            }
            metric_sources = {
                "ate_rmse_m": [
                    row for row in selected if row["ate_status"] == "ok"
                ],
                "rpe_translation_rmse_m": [
                    row
                    for row in selected
                    if row["rpe_translation_status"] == "ok"
                ],
                "rpe_rotation_deg_rmse": [
                    row
                    for row in selected
                    if row["rpe_rotation_status"] == "ok"
                ],
                "valid_pose_ratio": selected,
                "lost_events": selected,
                "mean_latency_ms": selected,
                "p95_latency_ms": selected,
            }
            for field, source in metric_sources.items():
                values = np.asarray(
                    [
                        float(row[field])
                        for row in source
                        if row.get(field) is not None
                    ],
                    dtype=float,
                )
                item[f"{field}_mean"] = (
                    float(values.mean()) if len(values) else None
                )
                item[f"{field}_std"] = (
                    float(values.std(ddof=1)) if len(values) > 1 else 0.0
                )
            aggregate.append(item)

    with (summary_root / "all_sequences_aggregate.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(aggregate[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(aggregate)
    (summary_root / "all_sequences_aggregate.json").write_text(
        json.dumps(aggregate, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    report = render_report(aggregate, rows)
    for report_path in (
        root / "experiments/A_baselines/available_sequences_report.md",
        root / "presentation/A/docs/available_sequences_report.md",
        root / "reports/A/available_sequences_report.md",
    ):
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(report, encoding="utf-8")
    print(summary_root / "all_sequences_metrics.csv")
    print(summary_root / "all_sequences_aggregate.csv")


if __name__ == "__main__":
    main()
