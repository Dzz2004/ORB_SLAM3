#!/usr/bin/env python3
"""Create representative trajectory plots for all available sequences."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import yaml

from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface


MODES = ("rgb", "rgb_imu", "rgbd_imu")
SEEDS = (0, 1, 2)


def read_tracking(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def segments(rows: list[dict[str, str]]) -> list[list[dict[str, str]]]:
    groups = []
    current = []
    map_id = None
    for row in rows:
        valid = row["valid"] == "1" and int(row["map_id"]) >= 0
        candidate_map = int(row["map_id"]) if valid else None
        if valid and (map_id is None or candidate_map == map_id):
            current.append(row)
            map_id = candidate_map
        else:
            if current:
                groups.append(current)
            current = [row] if valid else []
            map_id = candidate_map
    if current:
        groups.append(current)
    return groups


def slice_traj(
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


def representative(
    rows: list[dict[str, object]], run_root: Path, mode: str
) -> tuple[int, dict[str, object], Path]:
    selected = [row for row in rows if row["mode"] == mode]
    valid = [row for row in selected if row["status"] == "ok"]
    pool = valid or selected
    best = max(pool, key=lambda row: float(row["valid_pose_ratio"] or 0.0))
    return (
        int(best["seed"]),
        best,
        run_root / f"{mode}_seed{int(best['seed'])}",
    )


def plot_map_diagnostic(
    run_dir: Path, reference_path: Path, output: Path, title: str
) -> None:
    reference_full = file_interface.read_tum_trajectory_file(str(reference_path))
    estimated_full = file_interface.read_tum_trajectory_file(
        str(run_dir / "trajectory_frames.tum")
    )
    groups = segments(read_tracking(run_dir / "tracking.csv"))
    groups = [group for group in groups if len(group) >= 2]
    if not groups:
        raise ValueError(f"No plottable tracking segments in {run_dir}")

    columns = min(3, len(groups))
    rows = int(np.ceil(len(groups) / columns))
    figure, axes = plt.subplots(
        rows,
        columns,
        figsize=(5.6 * columns, 4.8 * rows),
        squeeze=False,
    )
    for axis, group in zip(axes.flat, groups, strict=False):
        start = float(group[0]["timestamp"])
        end = float(group[-1]["timestamp"])
        map_id = int(group[0]["map_id"])
        reference = slice_traj(reference_full, start, end)
        estimated = slice_traj(estimated_full, start, end)
        if len(reference.timestamps) and len(estimated.timestamps):
            try:
                reference, estimated = sync.associate_trajectories(
                    reference, estimated, max_diff=0.02
                )
            except Exception:
                pass
        axis.plot(
            reference.positions_xyz[:, 0],
            reference.positions_xyz[:, 1],
            color="#1f77b4",
            label="Ground truth",
        )
        axis.plot(
            estimated.positions_xyz[:, 0],
            estimated.positions_xyz[:, 1],
            color="#ff7f0e",
            label="Estimated",
        )
        axis.set_aspect("equal", adjustable="datalim")
        axis.set_title(f"Map {map_id}: {len(group)} tracked frames")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    for axis in axes.flat[len(groups) :]:
        axis.axis("off")
    figure.suptitle(title + " (per-frame diagnostic by map)")
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    plt.close(figure)


def plot_tracking_diagnostic(run_dir: Path, output: Path, title: str) -> None:
    rows = read_tracking(run_dir / "tracking.csv")
    timestamps = np.asarray([float(row["timestamp"]) for row in rows])
    valid = np.asarray([row["valid"] == "1" for row in rows], dtype=float)
    map_ids = np.asarray([int(row["map_id"]) for row in rows])
    figure, (coverage_axis, map_axis) = plt.subplots(
        2, 1, figsize=(8, 4.8), sharex=True
    )
    coverage_axis.step(
        timestamps,
        valid,
        where="post",
        color="#d62728",
        label="Tracking valid",
    )
    coverage_axis.set_ylim(-0.1, 1.1)
    coverage_axis.set_ylabel("Valid pose")
    coverage_axis.set_title(title + " (no usable primary trajectory)")
    coverage_axis.grid(True, alpha=0.3)
    coverage_axis.legend(loc="best")
    map_axis.step(
        timestamps,
        map_ids,
        where="post",
        color="#1f77b4",
        label="Active map ID",
    )
    map_axis.set_xlabel("time [s]")
    map_axis.set_ylabel("Map ID")
    map_axis.grid(True, alpha=0.3)
    map_axis.legend(loc="best")
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    plt.close(figure)


def contact_sheet(
    entries: list[dict[str, object]], title: str, output: Path
) -> None:
    sequences = []
    for entry in entries:
        if entry["sample_id"] not in sequences:
            sequences.append(entry["sample_id"])
    figure, axes = plt.subplots(
        len(sequences), len(MODES), figsize=(18, 6 * len(sequences))
    )
    for row, sample_id in enumerate(sequences):
        for column, mode in enumerate(MODES):
            axis = axes[row, column]
            entry = next(
                item
                for item in entries
                if item["sample_id"] == sample_id and item["mode"] == mode
            )
            if entry["plot_type"] == "primary_optimized_keyframe":
                axis.imshow(mpimg.imread(entry["figure_path"]))
                axis.axis("off")
            else:
                rows = read_tracking(
                    Path(entry["run_dir"]) / "tracking.csv"
                )
                timestamps = np.asarray(
                    [float(row["timestamp"]) for row in rows]
                )
                valid = np.asarray(
                    [row["valid"] == "1" for row in rows], dtype=float
                )
                axis.fill_between(
                    timestamps,
                    0,
                    valid,
                    step="post",
                    alpha=0.25,
                    color="#d62728",
                )
                axis.step(
                    timestamps,
                    valid,
                    where="post",
                    color="#d62728",
                )
                axis.set_ylim(-0.08, 1.12)
                axis.set_xlabel("time [s]")
                axis.set_ylabel("valid pose")
                axis.grid(True, alpha=0.3)
            short_status = {
                "ok": "primary trajectory available",
                "primary_trajectory_unavailable": "no primary trajectory",
                "invalid_map_discontinuity": "map discontinuity",
            }.get(str(entry["status"]), str(entry["status"]))
            axis.set_title(
                f"{sample_id} {mode} seed {entry['seed']}\n"
                f"coverage {100 * float(entry['valid_pose_ratio']):.1f}% | "
                f"KF {entry.get('keyframe_count', 'NA')} | {short_status}",
                fontsize=10,
            )
    figure.suptitle(title)
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=145)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument(
        "--matrix",
        type=Path,
        help="Sequence matrix; defaults to experiments/A_baselines/available_sequences.yaml.",
    )
    parser.add_argument(
        "--metrics",
        type=Path,
        help="all_sequences_metrics.json; defaults to ROOT/results/A/.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        help="Trajectory/contact-sheet output root; defaults to ROOT/results/A.",
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
    metrics_path = (
        args.metrics.resolve()
        if args.metrics
        else root / "results/A/all_sequences_metrics.json"
    )
    metrics = json.loads(
        metrics_path.read_text(encoding="utf-8")
    )
    result_output_root = (
        args.output_root.resolve()
        if args.output_root
        else root / "results/A"
    )
    output_root = result_output_root / "available_sequences/trajectories"
    output_root.mkdir(parents=True, exist_ok=True)
    entries = []

    for sequence in matrix["sequences"]:
        run_root = root / sequence["result_root"]
        reference = (
            root / sequence["input_root"] / "trajectory_gt.tum"
            if sequence.get("input_root")
            else root
            / "results/A/full_sequences/inputs"
            / sequence["id"]
            / "trajectory_gt.tum"
        )
        if not reference.exists():
            candidate = (
                root
                / "results/A/inputs_final/OldTownSummer"
                / sequence["difficulty"]
                / (
                    f"{sequence['trajectory']}_2p0s_9p9s"
                    if sequence["start_frame"] == 200
                    else f"{sequence['trajectory']}_start{sequence['start_frame']}"
                )
                / "trajectory_gt.tum"
            )
            if candidate.exists():
                reference = candidate
            elif sequence["id"] == "hard_P001":
                reference = (
                    root
                    / "results/A/inputs_final/OldTownSummer/hard/P001_start100"
                    / "trajectory_gt.tum"
                )
            elif sequence["id"] == "easy_P002":
                reference = (
                    root
                    / "results/A/validation/P002/easy/window_200_299"
                    / "trajectory_gt.tum"
                )
        sequence_rows = [
            row for row in metrics if row["sample_id"] == sequence["id"]
        ]
        for mode in MODES:
            seed, selected, run_dir = representative(
                sequence_rows, run_root, mode
            )
            output = output_root / f"{sequence['id']}_{mode}.png"
            overlay = run_dir / "trajectory_overlay.png"
            if selected["status"] == "ok" and overlay.exists():
                shutil.copy2(overlay, output)
                plot_type = "primary_optimized_keyframe"
            else:
                try:
                    plot_map_diagnostic(
                        run_dir,
                        reference,
                        output,
                        f"{sequence['id']} {mode} seed {seed}",
                    )
                    plot_type = "per_frame_by_map_diagnostic"
                except (OSError, ValueError):
                    plot_tracking_diagnostic(
                        run_dir,
                        output,
                        f"{sequence['id']} {mode} seed {seed}",
                    )
                    plot_type = "tracking_status_diagnostic"
            entries.append(
                {
                    "sample_id": sequence["id"],
                    "difficulty": sequence["difficulty"],
                    "trajectory": sequence["trajectory"],
                    "mode": mode,
                    "seed": seed,
                    "status": selected["status"],
                    "valid_pose_ratio": selected["valid_pose_ratio"],
                    "keyframe_count": selected.get("keyframe_count"),
                    "plot_type": plot_type,
                    "run_dir": str(run_dir),
                    "figure_path": str(output),
                }
            )

    manifest = result_output_root / "available_sequence_trajectory_manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(entries[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(entries)

    for difficulty in ("easy", "hard"):
        contact_sheet(
            [entry for entry in entries if entry["difficulty"] == difficulty],
            f"Available {difficulty} trajectories: 3 sequences x 3 modes",
            result_output_root
            / f"figures/{difficulty}_available_trajectories.png",
        )
    print(manifest)


if __name__ == "__main__":
    main()
