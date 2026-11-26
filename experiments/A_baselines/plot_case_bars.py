#!/usr/bin/env python3
"""Create presentation-ready Easy/Hard case bar charts from aggregate results."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


MODES = ("rgb", "rgb_imu", "rgbd_imu")
MODE_LABELS = ("RGB", "RGB+IMU", "RGB-D+IMU")
COLORS = ("#0072B2", "#E69F00", "#009E73")
CASES = ("P000", "P001", "P002")


def finite_or_none(value: object) -> float | None:
    if value is None:
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def csv_value(value: object) -> object:
    return "" if value is None else value


def add_metric_bars(
    axis: plt.Axes,
    rows: list[dict[str, object]],
    *,
    value_field: str,
    std_field: str,
    count_field: str,
    ylabel: str,
    title: str,
    scale: float = 1.0,
) -> None:
    positions = np.arange(len(MODES))
    for index, row in enumerate(rows):
        value = finite_or_none(row.get(value_field))
        count = int(row.get(count_field) or 0)
        if value is None or count == 0:
            axis.text(
                positions[index],
                0.03,
                f"N/A\n0/3",
                ha="center",
                va="bottom",
                fontsize=10,
                color="#555555",
            )
            continue
        std = finite_or_none(row.get(std_field)) or 0.0
        axis.bar(
            positions[index],
            value * scale,
            yerr=std * scale,
            capsize=4,
            color=COLORS[index],
            edgecolor="#222222",
            linewidth=0.8,
            error_kw={"elinewidth": 1.1, "ecolor": "#222222"},
        )
        axis.text(
            positions[index],
            value * scale,
            f"{value * scale:.3g}\n{count}/3",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    observed = [
        finite_or_none(row.get(value_field))
        for row in rows
        if finite_or_none(row.get(value_field)) is not None
    ]
    upper = max((value * scale for value in observed if value is not None), default=1.0)
    axis.set_ylim(0, max(upper * 1.32, 1.0 if scale == 1.0 else 0.1))
    axis.set_xticks(positions, MODE_LABELS)
    axis.set_ylabel(ylabel)
    axis.set_title(title, loc="left", fontweight="bold")
    axis.grid(axis="y", alpha=0.28)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)


def add_latency_panel(
    axis: plt.Axes,
    rows: list[dict[str, object]],
    title: str,
) -> None:
    positions = np.arange(len(MODES))
    width = 0.34
    means = [
        finite_or_none(row.get("mean_latency_ms_mean")) or 0.0
        for row in rows
    ]
    p95s = [
        finite_or_none(row.get("p95_latency_ms_mean")) or 0.0
        for row in rows
    ]
    axis.bar(
        positions - width / 2,
        means,
        width,
        color=COLORS,
        alpha=0.68,
        edgecolor="#222222",
        linewidth=0.7,
        label="Mean",
    )
    axis.bar(
        positions + width / 2,
        p95s,
        width,
        color=COLORS,
        edgecolor="#222222",
        linewidth=0.7,
        hatch="//",
        label="P95",
    )
    for position, mean, p95 in zip(positions, means, p95s, strict=True):
        axis.text(
            position - width / 2,
            mean,
            f"{mean:.0f}",
            ha="center",
            va="bottom",
            fontsize=8,
        )
        axis.text(
            position + width / 2,
            p95,
            f"{p95:.0f}",
            ha="center",
            va="bottom",
            fontsize=8,
        )
    axis.set_xticks(positions, MODE_LABELS)
    axis.set_ylabel("Latency [ms]")
    axis.set_title(title, loc="left", fontweight="bold")
    axis.set_ylim(0, max(p95s) * 1.25)
    axis.grid(axis="y", alpha=0.28)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(frameon=False, ncol=2, loc="upper left", fontsize=8)


def case_rows(
    aggregate: list[dict[str, object]], difficulty: str, case: str
) -> list[dict[str, object]]:
    rows = [
        row
        for row in aggregate
        if row["difficulty"] == difficulty
        and row["trajectory"] == case
    ]
    if len(rows) != len(MODES):
        raise ValueError(
            f"Expected three modes for {difficulty}/{case}, found {len(rows)}"
        )
    return sorted(rows, key=lambda row: MODES.index(str(row["mode"])))


def draw_case(
    aggregate: list[dict[str, object]],
    difficulty: str,
    case: str,
    destinations: list[Path],
) -> None:
    rows = case_rows(aggregate, difficulty, case)
    figure, axes = plt.subplots(
        2,
        2,
        figsize=(13.2, 8.2),
        layout="constrained",
    )
    add_metric_bars(
        axes[0, 0],
        rows,
        value_field="ate_rmse_m_mean",
        std_field="ate_rmse_m_std",
        count_field="ate_available_count",
        ylabel="ATE RMSE [m]",
        title="ATE (valid metric seeds)",
    )
    add_metric_bars(
        axes[0, 1],
        rows,
        value_field="rpe_translation_rmse_m_mean",
        std_field="rpe_translation_rmse_m_std",
        count_field="rpe_translation_available_count",
        ylabel="RPE translation RMSE [m]",
        title="RPE translation, 1 m interval",
    )
    add_metric_bars(
        axes[1, 0],
        rows,
        value_field="valid_pose_ratio_mean",
        std_field="valid_pose_ratio_std",
        count_field="run_count",
        ylabel="Valid-pose coverage [%]",
        title="Tracking coverage (all 3 seeds)",
        scale=100.0,
    )
    add_latency_panel(axes[1, 1], rows, "Track-call latency (all 3 seeds)")
    figure.suptitle(
        f"ORB-SLAM3 baseline: {difficulty.upper()} {case}",
        fontsize=17,
        fontweight="bold",
    )
    figure.text(
        0.995,
        0.985,
        "KF primary; mean ± SD; n = valid seeds; N/A ≠ 0",
        ha="right",
        va="top",
        fontsize=9,
        color="#444444",
    )
    stem = f"{difficulty}_{case}_bars"
    write_figure(figure, destinations, stem)


def draw_overview(
    aggregate: list[dict[str, object]],
    difficulty: str,
    destinations: list[Path],
) -> None:
    figure, axes = plt.subplots(
        len(CASES),
        4,
        figsize=(18.5, 11.5),
        layout="constrained",
    )
    for row_index, case in enumerate(CASES):
        rows = case_rows(aggregate, difficulty, case)
        add_metric_bars(
            axes[row_index, 0],
            rows,
            value_field="ate_rmse_m_mean",
            std_field="ate_rmse_m_std",
            count_field="ate_available_count",
            ylabel="ATE [m]",
            title="ATE",
        )
        add_metric_bars(
            axes[row_index, 1],
            rows,
            value_field="rpe_translation_rmse_m_mean",
            std_field="rpe_translation_rmse_m_std",
            count_field="rpe_translation_available_count",
            ylabel="RPE-T [m]",
            title="RPE translation",
        )
        add_metric_bars(
            axes[row_index, 2],
            rows,
            value_field="valid_pose_ratio_mean",
            std_field="valid_pose_ratio_std",
            count_field="run_count",
            ylabel="Coverage [%]",
            title="Tracking coverage",
            scale=100.0,
        )
        add_metric_bars(
            axes[row_index, 3],
            rows,
            value_field="p95_latency_ms_mean",
            std_field="p95_latency_ms_std",
            count_field="run_count",
            ylabel="P95 latency [ms]",
            title="Tracker P95 latency",
        )
        axes[row_index, 0].text(
            -0.28,
            1.03,
            case,
            transform=axes[row_index, 0].transAxes,
            fontsize=13,
            fontweight="bold",
            ha="left",
        )
    figure.suptitle(
        f"ORB-SLAM3 baseline: {difficulty.upper()} P000 / P001 / P002",
        fontsize=19,
        fontweight="bold",
    )
    figure.text(
        0.995,
        0.985,
        "KF primary; mean ± SD; n = valid seeds; N/A ≠ 0",
        ha="right",
        va="top",
        fontsize=10,
        color="#444444",
    )
    write_figure(figure, destinations, f"{difficulty}_three_cases_bars")


def write_figure(
    figure: plt.Figure,
    destinations: list[Path],
    stem: str,
) -> None:
    for directory in destinations:
        directory.mkdir(parents=True, exist_ok=True)
        figure.savefig(directory / f"{stem}.png", dpi=220, facecolor="white")
        figure.savefig(directory / f"{stem}.svg", facecolor="white")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    aggregate = json.loads(
        (root / "results/A/all_sequences_aggregate.json").read_text(
            encoding="utf-8"
        )
    )
    destinations = [
        root / "results/A/figures/case_bars",
        root / "presentation/A/figures/data",
    ]
    for difficulty in ("easy", "hard"):
        for case in CASES:
            draw_case(aggregate, difficulty, case, destinations)
        draw_overview(aggregate, difficulty, destinations)

    slide_rows = []
    for difficulty in ("easy", "hard"):
        for case in CASES:
            for row in case_rows(aggregate, difficulty, case):
                slide_rows.append(
                    {
                        "difficulty": difficulty,
                        "case": case,
                        "mode": row["mode"],
                        "ate_rmse_m": csv_value(row.get("ate_rmse_m_mean")),
                        "ate_sd_m": (
                            csv_value(row.get("ate_rmse_m_std"))
                            if row["ate_available_count"] > 0
                            else ""
                        ),
                        "ate_n": row["ate_available_count"],
                        "rpe_translation_rmse_m": csv_value(
                            row.get("rpe_translation_rmse_m_mean")
                        ),
                        "rpe_translation_sd_m": csv_value(
                            row.get("rpe_translation_rmse_m_std")
                        )
                        if row["rpe_translation_available_count"] > 0
                        else "",
                        "rpe_translation_n": row[
                            "rpe_translation_available_count"
                        ],
                        "coverage_percent": (
                            100.0 * float(row["valid_pose_ratio_mean"])
                        ),
                        "coverage_sd_percent": (
                            100.0 * float(row["valid_pose_ratio_std"])
                        ),
                        "runs": row["run_count"],
                        "mean_latency_ms": row["mean_latency_ms_mean"],
                        "p95_latency_ms": row["p95_latency_ms_mean"],
                    }
                )
    for destination in destinations:
        with (destination / "slide_data.csv").open(
            "w", newline="", encoding="utf-8"
        ) as handle:
            writer = csv.DictWriter(
                handle, fieldnames=list(slide_rows[0]), lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(slide_rows)

    manifest = {
        "source": "results/A/all_sequences_aggregate.json",
        "aggregation": (
            "Mean and sample SD over valid metric seeds; coverage and latency use "
            "all three seeds."
        ),
        "missing_values": "Rendered as N/A; never converted to zero.",
        "primary_trajectory": "optimized-keyframe body/NED trajectory",
        "uncertainty": "sample standard deviation across valid seeds",
        "latency_scope": "Track() call only",
        "outputs": [
            f"{difficulty}_{case}_bars"
            for difficulty in ("easy", "hard")
            for case in CASES
        ]
        + ["easy_three_cases_bars", "hard_three_cases_bars"],
    }
    for directory in destinations:
        (directory / "figure_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
