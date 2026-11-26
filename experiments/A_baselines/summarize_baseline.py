#!/usr/bin/env python3
"""Summarize A baseline and holdout runs and create comparison figures."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


MODES = ("rgb", "rgb_imu", "rgbd_imu")
SEEDS = (0, 1, 2)
COLORS = ("#4C78A8", "#F58518", "#54A24B")
METRIC_FIELDS = (
    "ate_rmse_m",
    "rpe_translation_rmse_m",
    "rpe_rotation_rmse_deg",
    "orientation_ape_rmse_deg",
    "valid_pose_ratio",
    "lost_events",
    "map_switches",
    "longest_loss_frames",
    "mean_latency_ms",
    "p95_latency_ms",
)
ERROR_FIELDS = {
    "ate_rmse_m",
    "rpe_translation_rmse_m",
    "rpe_rotation_rmse_deg",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def tracking_metrics(path: Path) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    valid_rows = [
        row
        for row in rows
        if row.get("valid") == "1" and int(row.get("map_id", "-1")) >= 0
    ]
    map_ids = sorted({int(row["map_id"]) for row in valid_rows})
    map_switches = 0
    previous_map = None
    current_loss_run = 0
    longest_loss_run = 0
    for row in rows:
        if row.get("valid") == "1" and int(row.get("map_id", "-1")) >= 0:
            current_map = int(row["map_id"])
            if previous_map is not None and current_map != previous_map:
                map_switches += 1
            previous_map = current_map
            current_loss_run = 0
        else:
            current_loss_run += 1
            longest_loss_run = max(longest_loss_run, current_loss_run)
    return {
        "map_ids_in_valid_tracking": map_ids,
        "unique_map_count": len(map_ids),
        "map_switches": map_switches,
        "longest_loss_frames": longest_loss_run,
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = []
    for row in rows:
        for field in row:
            if field not in fieldnames:
                fieldnames.append(field)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=fieldnames, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def load_run(
    run_dir: Path,
    *,
    trajectory: str,
    difficulty: str,
    mode: str,
    seed: int,
    window: str,
) -> dict[str, object]:
    metrics = json.loads(
        (run_dir / "metrics.json").read_text(encoding="utf-8")
    )
    meta_path = run_dir / "run_meta.json"
    meta = (
        json.loads(meta_path.read_text(encoding="utf-8"))
        if meta_path.exists()
        else {}
    )
    settings = run_dir / "settings_snapshot.yaml"
    tracking = tracking_metrics(run_dir / "tracking.csv")
    return {
        "dataset": "TartanAirV2",
        "environment": "OldTownSummer",
        "trajectory": trajectory,
        "difficulty": difficulty,
        "window": window,
        "mode": mode,
        "seed": seed,
        "run_status": meta.get("status", "unknown"),
        "metric_status": metrics.get("status"),
        "ate_rmse_m": metrics.get("ate_rmse"),
        "rpe_translation_rmse_m": metrics.get("rpe_translation_rmse"),
        "rpe_rotation_rmse_deg": metrics.get("rpe_rotation_deg_rmse"),
        "orientation_ape_rmse_deg": metrics.get(
            "orientation_ape_rmse_deg"
        ),
        "valid_pose_frames": metrics.get("valid_pose_frames"),
        "valid_pose_ratio": metrics.get("valid_pose_ratio"),
        "lost_events": metrics.get("lost_events"),
        "matched_pose_frames": metrics.get("matched_pose_frames"),
        **tracking,
        "mean_latency_ms": metrics.get("mean_latency_ms"),
        "p95_latency_ms": metrics.get("p95_latency_ms"),
        "alignment": metrics.get("alignment"),
        "evaluated_map_id": metrics.get("evaluated_map_id"),
        "evaluated_map_valid_rows": metrics.get(
            "evaluated_map_valid_rows"
        ),
        "evaluated_window_start_s": metrics.get("evaluated_window_start"),
        "evaluated_window_end_s": metrics.get("evaluated_window_end"),
        "git_commit": meta.get("git_commit", "unknown"),
        "config_sha256": sha256(settings) if settings.exists() else None,
        "trajectory_source": meta.get("trajectory_source"),
        "optimized_trajectory_source": meta.get(
            "optimized_trajectory_source"
        ),
    }


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    aggregate = []
    for mode in MODES:
        selected = [row for row in rows if row["mode"] == mode]
        valid_metrics = [
            row for row in selected if row.get("metric_status") == "ok"
        ]
        summary = {
            "difficulty": selected[0]["difficulty"],
            "trajectory": selected[0]["trajectory"],
            "mode": mode,
            "seed_count": len(selected),
            "metric_ok_count": len(valid_metrics),
        }
        for field in METRIC_FIELDS:
            source = valid_metrics if field in ERROR_FIELDS else selected
            values = np.asarray(
                [
                    float(row[field])
                    for row in source
                    if row.get(field) is not None
                ],
                dtype=float,
            )
            summary[f"{field}_mean"] = (
                float(values.mean()) if len(values) else None
            )
            summary[f"{field}_std"] = (
                float(values.std(ddof=1)) if len(values) > 1 else 0.0
            )
        aggregate.append(summary)
    return aggregate


def plot_summary(
    aggregate: list[dict[str, object]],
    *,
    title: str,
    output: Path,
) -> None:
    figure, axes = plt.subplots(2, 3, figsize=(15, 8))
    panels = (
        ("ate_rmse_m", "ATE RMSE [m]"),
        ("rpe_translation_rmse_m", "RPE translation [m]"),
        ("rpe_rotation_rmse_deg", "RPE rotation [deg]"),
        ("valid_pose_ratio", "Valid pose coverage"),
        ("mean_latency_ms", "Mean latency [ms]"),
        ("p95_latency_ms", "P95 latency [ms]"),
    )
    x = np.arange(len(MODES))
    for axis, (field, label) in zip(axes.flat, panels, strict=True):
        means = np.asarray(
            [
                row[f"{field}_mean"]
                if row[f"{field}_mean"] is not None
                else np.nan
                for row in aggregate
            ],
            dtype=float,
        )
        if field == "valid_pose_ratio":
            means *= 100.0
            label = "Valid pose coverage [%]"
        stds = np.asarray(
            [
                row[f"{field}_std"]
                if row[f"{field}_std"] is not None
                else 0.0
                for row in aggregate
            ],
            dtype=float,
        )
        if field == "valid_pose_ratio":
            stds *= 100.0
        errors = np.vstack(
            [
                np.where(np.isnan(means), 0.0, np.minimum(means, stds)),
                stds,
            ]
        )
        axis.bar(x, means, yerr=errors, capsize=4, color=COLORS)
        axis.set_xticks(x, MODES)
        axis.set_ylabel(label)
        axis.grid(True, axis="y", alpha=0.3)
        if field == "valid_pose_ratio":
            axis.set_ylim(0, 105)
    figure.suptitle(title)
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=170)
    plt.close(figure)


def plot_hard_robustness(
    aggregate: list[dict[str, object]], output: Path
) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(12, 8))
    panels = (
        ("valid_pose_ratio", "Valid pose coverage [%]", 100.0),
        ("lost_events", "Loss events", 1.0),
        ("map_switches", "Map switches", 1.0),
        ("p95_latency_ms", "P95 latency [ms]", 1.0),
    )
    x = np.arange(len(MODES))
    for axis, (field, label, scale) in zip(axes.flat, panels, strict=True):
        means = np.asarray(
            [
                row[f"{field}_mean"]
                if row[f"{field}_mean"] is not None
                else np.nan
                for row in aggregate
            ],
            dtype=float,
        ) * scale
        stds = np.asarray(
            [
                row[f"{field}_std"]
                if row[f"{field}_std"] is not None
                else 0.0
                for row in aggregate
            ],
            dtype=float,
        ) * scale
        errors = np.vstack(
            [
                np.where(np.isnan(means), 0.0, np.minimum(means, stds)),
                stds,
            ]
        )
        axis.bar(x, means, yerr=errors, capsize=4, color=COLORS)
        axis.set_xticks(x, MODES)
        axis.set_ylabel(label)
        axis.grid(True, axis="y", alpha=0.3)
        if field == "valid_pose_ratio":
            axis.set_ylim(0, 105)
    figure.suptitle("A hard P001 robustness, 3 seeds, 10.0-19.9 s")
    figure.tight_layout()
    figure.savefig(output, dpi=170)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(
            os.environ.get("A_REPO_ROOT", Path(__file__).resolve().parents[2])
        ),
    )
    args = parser.parse_args()
    root = args.root.resolve()
    result_root = root / "results/A"
    figure_root = result_root / "figures"
    figure_root.mkdir(parents=True, exist_ok=True)

    baseline_rows = []
    baseline_root = root / "results/A/baseline_final/easy/P000_window_200_299"
    for mode in MODES:
        for seed in SEEDS:
            baseline_rows.append(
                load_run(
                    baseline_root / f"{mode}_seed{seed}",
                    trajectory="P000",
                    difficulty="easy",
                    mode=mode,
                    seed=seed,
                    window="20.0-29.9s",
                )
            )
    baseline_aggregate = aggregate_rows(baseline_rows)
    write_csv(result_root / "baseline_metrics.csv", baseline_rows)
    (result_root / "baseline_metrics.json").write_text(
        json.dumps(baseline_rows, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_csv(result_root / "baseline_aggregate.csv", baseline_aggregate)
    (result_root / "baseline_aggregate.json").write_text(
        json.dumps(baseline_aggregate, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    plot_summary(
        baseline_aggregate,
        title="A Baseline: TartanAir easy P000, 3 seeds, 20.0-29.9 s",
        output=figure_root / "baseline_summary.png",
    )

    validation_rows = []
    validation_root = root / "results/A/validation/P002/easy"
    for mode in MODES:
        for seed in SEEDS:
            validation_rows.append(
                load_run(
                    validation_root / f"{mode}_seed{seed}",
                    trajectory="P002",
                    difficulty="easy",
                    mode=mode,
                    seed=seed,
                    window="20.0-29.9s",
                )
            )
    validation_aggregate = aggregate_rows(validation_rows)
    write_csv(result_root / "validation_metrics.csv", validation_rows)
    (result_root / "validation_metrics.json").write_text(
        json.dumps(validation_rows, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_csv(
        result_root / "validation_aggregate.csv", validation_aggregate
    )
    (result_root / "validation_aggregate.json").write_text(
        json.dumps(validation_aggregate, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    plot_summary(
        validation_aggregate,
        title="A Holdout: TartanAir easy P002, 3 seeds, 20.0-29.9 s",
        output=figure_root / "validation_summary.png",
    )

    hard_rows = []
    hard_root = root / "results/A/baseline_final/hard/P001_window_start100"
    for mode in MODES:
        for seed in SEEDS:
            hard_rows.append(
                load_run(
                    hard_root / f"{mode}_seed{seed}",
                    trajectory="P001",
                    difficulty="hard",
                    mode=mode,
                    seed=seed,
                    window="10.0-19.9s",
                )
            )
    hard_aggregate = aggregate_rows(hard_rows)
    write_csv(result_root / "hard_metrics.csv", hard_rows)
    (result_root / "hard_metrics.json").write_text(
        json.dumps(hard_rows, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_csv(result_root / "hard_aggregate.csv", hard_aggregate)
    (result_root / "hard_aggregate.json").write_text(
        json.dumps(hard_aggregate, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    plot_hard_robustness(
        hard_aggregate, figure_root / "hard_robustness.png"
    )

    print(f"Wrote {result_root / 'baseline_metrics.csv'}")
    print(f"Wrote {result_root / 'baseline_aggregate.csv'}")
    print(f"Wrote {result_root / 'validation_metrics.csv'}")
    print(f"Wrote {result_root / 'validation_aggregate.csv'}")
    print(f"Wrote {result_root / 'hard_metrics.csv'}")
    print(f"Wrote {result_root / 'hard_aggregate.csv'}")
    print(f"Wrote {figure_root / 'baseline_summary.png'}")
    print(f"Wrote {figure_root / 'validation_summary.png'}")
    print(f"Wrote {figure_root / 'hard_robustness.png'}")


if __name__ == "__main__":
    main()
