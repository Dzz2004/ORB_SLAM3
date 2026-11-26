"""Run E's reproducible fixture matrix or replay one recorded case."""

import argparse
import hashlib
import json
import re
import shutil
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy

from degradation_policy import ImuPredictor
from replay import VARIANTS, evaluate, read_case, replay, write_csv, write_tum
from synthetic import CASES, SEEDS, make_case


CONFIG = Path(__file__).with_name("config.json")
STATE_COLORS = {"NORMAL": "#25785e", "VISION_DEGRADED": "#d18b18", "DEPTH_DEGRADED": "#2b75af",
                "IMU_ONLY": "#7850ad", "RECOVERY": "#ba466c"}
LABELS = {"baseline": "Raw SLAM", "monitor": "Quality monitor", "monitor_imu": "Monitor + IMU"}
COLORS = ("#53677a", "#227d73", "#7850ad")


def plot_timeline(path, timelines, metadata):
    fig, axes = plt.subplots(len(timelines), 1, figsize=(10.4, 2.15 * len(timelines)), squeeze=False,
                             layout="constrained")
    for ax, (label, rows, faults) in zip(axes[:, 0], timelines):
        timestamps = [row["timestamp"] for row in rows]
        for _, start, end in faults:
            ax.axvspan(start, end, color="#efdfb8", alpha=0.55)
        for state, color in STATE_COLORS.items():
            selected = [row["timestamp"] for row in rows if row["state"] == state]
            ax.scatter(selected, [1] * len(selected), s=18, color=color, label=state)
        for source, y, color in (("slam", 0, "#13746d"), ("imu", 0, "#7850ad"), ("none", 0, "#bd4b4b")):
            selected = [row["timestamp"] for row in rows if row["source"] == source]
            ax.scatter(selected, [y] * len(selected), s=14, color=color)
        ax.set(xlim=(timestamps[0] - 0.1, timestamps[-1] + 0.1), ylim=(-0.42, 1.42),
               yticks=[0, 1], yticklabels=["Output", "State"], title=label)
        ax.grid(axis="x", alpha=0.18)
    axes[-1, 0].set_xlabel("Time (s)   •   Output: SLAM (green), IMU (purple), missing (red)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center",
               ncols=5, frameon=False, fontsize=8)
    fig.suptitle(f"State and pose-source timeline  |  {metadata}", fontsize=13)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def plot_ablation(path, summaries, metadata):
    labels = list(dict.fromkeys(row["fault"] for row in summaries))
    panels = (("output_coverage", "Pose output coverage (%)", 100),
              ("normal_frame_accuracy", "Normal-frame accuracy (%)", 100),
              ("prediction_coverage_s", "IMU prediction coverage (s)", 1),
              ("recovery_mean_s", "Recovery time, recovered trials (s)", 1))
    fig, axes = plt.subplots(2, 2, figsize=(10.4, 6.6), layout="constrained")
    for ax, (key, title, scale) in zip(axes.flat, panels):
        x = np.arange(len(labels))
        for offset, (variant, color) in enumerate(zip(VARIANTS, COLORS)):
            groups = [[row[key] * scale for row in summaries
                       if row["fault"] == label and row["variant"] == variant and row[key] is not None]
                      for label in labels]
            centres = x + (offset - 1) * 0.24
            means = [float(np.mean(group)) if group else np.nan for group in groups]
            spread = [float(np.std(group, ddof=1)) if len(group) > 1 else 0 for group in groups]
            ax.bar(centres, means, width=0.22, yerr=spread, capsize=2,
                   color=color, label=LABELS[variant])
            for centre, mean in zip(centres, means):
                if np.isnan(mean):
                    ax.annotate("n/a", (centre, 0), xytext=(0, 3), textcoords="offset points",
                                ha="center", fontsize=7)
        ax.set(title=title, xticks=x, xticklabels=[label.replace("_", " ") for label in labels])
        if scale == 100:
            ax.set_ylim(0, 110)
        ax.grid(axis="y", alpha=0.2)
        ax.set_axisbelow(True)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncols=3, frameon=False)
    fig.suptitle(f"Policy ablation  |  {metadata}\nMean ± sample SD where repeated", fontsize=13)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def run(args, *, matrix=None, predictor_factory=ImuPredictor, provenance=None):
    config = json.loads(args.config.read_text(encoding="utf-8-sig"))
    out = args.out
    figures = out / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    if matrix is not None:
        inputs = matrix
        ref = f"{args.sequence}  •  log-level masking"
    elif args.demo:
        inputs = [(fault, seed, args.demo / fault / str(seed)) for fault in CASES for seed in SEEDS]
        ref = "synthetic fixture"
    else:
        inputs = [(args.fault, args.seed, args.case)]
        ref = f"{args.dataset} / {args.sequence}  •  seed {args.seed}"
    replicated = args.demo or matrix is not None
    ablation, all_events, recovery_rows, timelines, hashes = [], [], [], [], {}
    for fault, seed, folder in inputs:
        if args.demo:
            make_case(folder, fault, seed)
        slug = f"{fault}_seed{seed}"
        frames, samples, faults = read_case(folder, config)
        run_folder = out / "runs" / slug
        run_folder.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(folder / "tracking.csv", run_folder / "tracking.csv")
        hashes[slug] = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                        for path in (folder / name for name in
                                     ("tracking.csv", "imu.csv", "trajectory_est.tum", "trajectory_gt.tum", "faults.csv"))
                        if path.is_file()}
        for variant in VARIANTS:
            start = time.perf_counter()
            decisions = replay(frames, samples, variant, config, predictor_factory)
            elapsed = time.perf_counter() - start
            metrics, events, recovery = evaluate(frames, decisions, faults, config)
            recovery_times = [row["recovery_s"] for row in recovery if row["recovered"]]
            run_id = dict(fault=fault, seed=seed, variant=variant)
            ablation.append(dict(**run_id, **metrics,
                                 recovery_mean_s=float(np.mean(recovery_times)) if recovery_times else None,
                                 replay_ms_per_frame=elapsed * 1000 / len(frames)))
            all_events.extend(dict(**run_id, **{k: v for k, v in event.items() if k != "fault"},
                                   active_fault=event["fault"]) for event in events)
            recovery_rows.extend(dict(case=fault, seed=seed, variant=variant, **row) for row in recovery)
            write_tum(run_folder / f"{variant}.tum",
                      ((frame.time, decision[0].pose) for frame, decision in zip(frames, decisions)))
            if variant == "monitor_imu" and (not replicated or seed == 0):
                timelines.append((fault.replace("_", " "), events, faults))
    write_csv(out / "ablation.csv", ablation)
    write_csv(out / "policy_events.csv", all_events)
    write_csv(out / "recovery.csv", recovery_rows,
              ["case", "seed", "variant", "fault", "start_s", "end_s", "recovery_s", "recovered", "prediction_in_fault_s"])
    dataset, sequence = ("synthetic_fixture", "analytic_6dof") if args.demo else (args.dataset, args.sequence)
    payload = dict(dataset=dataset, sequence=sequence, revision=args.revision, config=config, input_sha256=hashes,
                   versions=dict(python=sys.version.split()[0], numpy=np.__version__,
                                 scipy=scipy.__version__, matplotlib=matplotlib.__version__),
                   pose_frame="shared metric world frame; input poses already registered",
                   runs=ablation)
    if provenance is not None:
        payload["provenance"] = provenance
    (out / "metrics.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                                      encoding="utf-8", newline="\n")
    plot_timeline(figures / "state_timeline.png", timelines, f"{ref}  •  seed 0" if replicated else ref)
    plot_ablation(figures / "ablation.png", ablation, f"{ref}  •  seeds 0, 1, 2" if replicated else ref)
    print(f"{len(ablation)} replays, {len(all_events)} frame decisions in {out}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--demo", type=Path, metavar="INPUT_DIR", help="Generate and evaluate the nine synthetic inputs")
    mode.add_argument("--case", type=Path, help="Replay one directory of recorded inputs")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--dataset")
    parser.add_argument("--sequence")
    parser.add_argument("--fault")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--revision", help="Git revision that produced the recording or fixture")
    args = parser.parse_args()
    if args.case and any(value is None for value in (args.dataset, args.sequence, args.fault, args.seed)):
        parser.error("--case requires --dataset, --sequence, --fault and --seed")
    if args.case and not re.fullmatch(r"[A-Za-z0-9_-]+", args.fault):
        parser.error("--fault must use letters, digits, underscores or hyphens")
    if args.case and args.out.resolve() == args.case.resolve():
        parser.error("--out must differ from the input folder")
    run(args)


if __name__ == "__main__":
    main()
