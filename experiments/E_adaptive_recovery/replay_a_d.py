"""Replay A's normal logs and controlled log-level masks with D's predictor."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
from urllib.request import urlopen

import numpy as np
from scipy.spatial.transform import Rotation

from d_predictor import DImuPredictor
from degradation_policy import Pose
from replay import check_times, read_csv, write_csv, write_tum
from run import CONFIG, plt, run


A_REVISION = "ffe81540c2c9f8ad4302f505a1695332ce090fe6"
D_REVISION = "5a888ab1f33ab5f0e8e51c3a664ac6fd6bb2960c"
A_URL = f"https://raw.githubusercontent.com/BitStrawber/ORB_SLAM3/{A_REVISION}/results/A"
A_RUN = "baseline_final/easy/P000_window_200_299/rgbd_imu_seed"
GT_PATH = "inputs_final/OldTownSummer/easy/P000_2p0s_9p9s/trajectory_gt.tum"
SEEDS = (0, 1, 2)
CASES = ("clean", "depth_mask", "rgb_mask", "rgb_depth_mask")
START, STOP = 23.0, 29.9
FAULT_START, FAULT_STOP = 25.0, 27.0


def source_paths(root):
    return {root / "common" / "trajectory_gt.tum": f"{A_URL}/{GT_PATH}"} | {
        root / "A" / f"seed{seed}" / name: f"{A_URL}/{A_RUN}{seed}/{name}"
        for seed in SEEDS for name in ("tracking.csv", "trajectory_frames.tum", "imu_snapshot.csv")
    }


def download_inputs(paths):
    for path, url in paths.items():
        with urlopen(url, timeout=45) as response:
            data = response.read()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)


def keyed_trajectory(path):
    rows = np.loadtxt(path, ndmin=2)
    times = np.round(rows[:, 0], 5)
    check_times(times, path)
    return {t: Pose(row[1:4], Rotation.from_quat(row[4:8])) for t, row in zip(times, rows)}


def align_reference(estimates, truth):
    # This transform is applied only to evaluator truth. Policy inputs stay in SLAM's frame.
    common = [t for t in estimates if START <= t < FAULT_START and t in truth]
    if len(common) < 10:
        raise ValueError("Insufficient paired warm-up poses to align the reference")
    rotations = Rotation.concatenate([estimates[t].rotation * truth[t].rotation.inv() for t in common])
    rotation = rotations.mean()
    translation = np.mean([estimates[t].position - rotation.apply(truth[t].position) for t in common], axis=0)
    aligned = {t: Pose(rotation.apply(pose.position) + translation, rotation * pose.rotation)
               for t, pose in truth.items() if START <= t <= STOP}
    return aligned, dict(fit_frames=len(common), rotation_xyzw=rotation.as_quat().tolist(),
                         translation_m=translation.tolist())


def prepare_cases(raw_root, cases_root):
    truth = keyed_trajectory(raw_root / "common" / "trajectory_gt.tum")
    matrix, alignment = [], {}
    for seed in SEEDS:
        source = raw_root / "A" / f"seed{seed}"
        estimates = keyed_trajectory(source / "trajectory_frames.tum")
        aligned_gt, alignment[seed] = align_reference(estimates, truth)
        observations = []
        for row in read_csv(source / "tracking.csv"):
            timestamp = round(float(row["timestamp"]), 5)
            if START <= timestamp <= STOP:
                # A exports the count of tracked keypoints, rather than a normalized quality score.
                count = int(row["quality_rgb"])
                observations.append(dict(row, timestamp=timestamp, tracked_keypoints=count,
                                         quality_rgb=min(1.0, count / 1250)))
        check_times([row["timestamp"] for row in observations], "A tracking")
        if len(observations) != 70 or len({row["map_id"] for row in observations}) != 1:
            raise ValueError("Expected 70 frames in a single map in A's post-initialization slice")
        times = [row["timestamp"] for row in observations]
        if any(t not in aligned_gt for t in times):
            raise ValueError("A reference does not cover the selected time window")

        imu = [row for row in read_csv(source / "imu_snapshot.csv")
               if START - 0.05 <= float(row["timestamp"]) <= STOP + 0.05]
        imu_times = [float(row["timestamp"]) for row in imu]
        check_times(imu_times, "A IMU")
        if not imu_times or imu_times[0] > START or imu_times[-1] < STOP:
            raise ValueError("A IMU snapshot does not cover the full time window")
        if not all(math.isfinite(float(value)) for row in imu for value in row.values()):
            raise ValueError("A IMU sample is incomplete or non-finite")

        for case in CASES:
            folder = cases_root / case / str(seed)
            folder.mkdir(parents=True, exist_ok=True)
            rows, poses, faults = [], [], []
            if case != "clean":
                faults.append(dict(fault=case, start_s=FAULT_START, end_s=FAULT_STOP))
            for observation in observations:
                row = observation.copy()
                t = row["timestamp"]
                if FAULT_START <= t < FAULT_STOP:
                    if case in {"rgb_mask", "rgb_depth_mask"}:
                        row.update(valid=0, state="LOST", source="none", quality_rgb=0, tracked_keypoints=0)
                    if case in {"depth_mask", "rgb_depth_mask"}:
                        row["quality_depth"] = 0
                rows.append(row)
                if int(row["valid"]) and t in estimates:
                    poses.append((t, estimates[t]))
            write_csv(folder / "tracking.csv", rows)
            write_csv(folder / "imu.csv", imu)
            write_csv(folder / "faults.csv", faults, ["fault", "start_s", "end_s"])
            write_tum(folder / "trajectory_est.tum", poses)
            write_tum(folder / "trajectory_gt.tum", ((t, aligned_gt[t]) for t in times))
            matrix.append((case, seed, folder))
    return sorted(matrix, key=lambda item: (CASES.index(item[0]), item[1])), alignment


def summarize_prediction(folder):
    rows = [row for row in read_csv(folder / "policy_events.csv")
            if row["fault"] == "rgb_depth_mask" and row["source"] == "imu"]
    fields = ("seed", "prediction_age_s", "position_error_m", "rotation_error_deg", "within_error_limit")
    write_csv(folder / "prediction_error.csv", [{key: row[key] for key in fields} for row in rows], fields)
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 3.9), layout="constrained")
    for ax, key, label in zip(axes, fields[2:4], ("Position error (m)", "Rotation error (deg)")):
        for seed in SEEDS:
            group = [row for row in rows if int(row["seed"]) == seed]
            ax.plot([float(row["prediction_age_s"]) for row in group],
                    [float(row[key]) for row in group], label=f"Seed {seed}", linewidth=2)
        ax.set(xlabel="Prediction age (s)", ylabel=label)
        ax.grid(alpha=.22)
    limit = json.loads((folder / "metrics.json").read_text(encoding="utf-8"))["config"]["evaluation_position_m"]
    axes[0].axhline(limit, color="#bd4b4b", linestyle="--", linewidth=1, label=f"{limit:g} m evaluation threshold")
    axes[0].legend(loc="upper left", fontsize=8)
    fig.suptitle("D integrator with E initialization  |  OldTownSummer easy/P000 log-level masking")
    fig.savefig(folder / "figures" / "prediction_error.png", dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--download", action="store_true", help="Retrieve the pinned A PR files")
    parser.add_argument("--config", type=Path, default=CONFIG)
    args = parser.parse_args()
    paths = source_paths(args.raw)
    if args.download:
        download_inputs(paths)
    matrix, alignment = prepare_cases(args.raw, args.cases)
    provenance = dict(
        evidence="A's original baseline tracking plus policy-input masks; no rerun of sensor corruption in SLAM",
        a_commit=A_REVISION, d_commit=D_REVISION,
        sample_window_s=[START, STOP], mask_window_s=[FAULT_START, FAULT_STOP],
        evaluation_alignment_warmup_s=[START, FAULT_START],
        alignment_truth_to_slam=alignment,
        predictor_initialization="two last E-accepted frame poses; configured zero biases and SLAM-world gravity",
        raw_files={path.relative_to(args.raw).as_posix():
                   dict(url=url, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                   for path, url in paths.items()})
    settings = SimpleNamespace(config=args.config, out=args.out, demo=None, case=None,
                               dataset="TartanAir V2", sequence="OldTownSummer easy/P000",
                               revision=A_REVISION)
    run(settings, matrix=matrix, predictor_factory=DImuPredictor, provenance=provenance)
    summarize_prediction(args.out)


if __name__ == "__main__":
    main()
