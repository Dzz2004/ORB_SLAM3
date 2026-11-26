#!/usr/bin/env python3
"""Create a presentation overview for baseline sensor-degradation risks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


def find_rows(
    aggregate: list[dict[str, object]],
    difficulty: str,
    mode: str,
) -> list[dict[str, object]]:
    return [
        row
        for row in aggregate
        if row["difficulty"] == difficulty and row["mode"] == mode
    ]


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
    hard_rgb = find_rows(aggregate, "hard", "rgb")
    hard_rgb_imu = find_rows(aggregate, "hard", "rgb_imu")
    hard_rgbd_imu = find_rows(aggregate, "hard", "rgbd_imu")

    p001_rgb = next(row for row in hard_rgb if row["trajectory"] == "P001")
    imu_coverage = [
        100.0 * float(row["valid_pose_ratio_mean"]) for row in hard_rgb_imu
    ]
    rgbd_coverage = [
        100.0 * float(row["valid_pose_ratio_mean"]) for row in hard_rgbd_imu
    ]
    imu_primary = sum(
        int(row["primary_trajectory_available_count"])
        for row in hard_rgb_imu
    )
    rgbd_primary = sum(
        int(row["primary_trajectory_available_count"])
        for row in hard_rgbd_imu
    )

    plt.rcParams.update(
        {
            "font.family": "Microsoft YaHei",
            "font.size": 12,
            "axes.unicode_minus": False,
        }
    )
    figure = plt.figure(figsize=(16, 9), facecolor="#F7F7F5")
    axis = figure.add_axes([0, 0, 1, 1])
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)
    axis.axis("off")

    figure.text(
        0.055,
        0.93,
        "ORB-SLAM3 基线面临的传感器退化问题",
        fontsize=28,
        fontweight="bold",
        color="#17202A",
        ha="left",
        va="top",
    )
    figure.text(
        0.055,
        0.865,
        "三传感器可以互补，但任一侧观测退化仍会传导为失跟、初始化失败或地图重置。",
        fontsize=16,
        color="#4D5656",
        ha="left",
        va="top",
    )

    cards = [
        {
            "x": 0.055,
            "title": "RGB：视觉几何",
            "color": "#0072B2",
            "light": "#E8F2F8",
            "strong": "纹理丰富时提供主要特征与长期约束",
            "risk": "弱纹理、运动模糊、光照变化、动态遮挡",
            "evidence": (
                f"Hard P001 覆盖率 {100.0 * float(p001_rgb['valid_pose_ratio_mean']):.1f}%\n"
                f"连续 ATE/RPE 可用 {p001_rgb['ate_available_count']}/3"
            ),
        },
        {
            "x": 0.372,
            "title": "IMU：短时运动",
            "color": "#E69F00",
            "light": "#FFF4D9",
            "strong": "高频角速度/加速度，可在短时预测运动",
            "risk": "初始化不足、bias 未收敛、积分漂移",
            "evidence": (
                f"Hard RGB+IMU 覆盖率 {min(imu_coverage):.1f}%–{max(imu_coverage):.1f}%\n"
                f"可用主轨迹 {imu_primary}/9"
            ),
        },
        {
            "x": 0.689,
            "title": "Depth：尺度与几何",
            "color": "#009E73",
            "light": "#E7F7F1",
            "strong": "提供公制尺度和近距三维约束",
            "risk": "空洞、噪声、无效像素、连续深度缺失",
            "evidence": (
                f"Hard RGB-D+IMU 覆盖率 {min(rgbd_coverage):.1f}%–{max(rgbd_coverage):.1f}%\n"
                f"可用主轨迹 {rgbd_primary}/9"
            ),
        },
    ]
    for card in cards:
        x = float(card["x"])
        axis.add_patch(
            FancyBboxPatch(
                (x, 0.37),
                0.255,
                0.41,
                boxstyle="round,pad=0.012,rounding_size=0.018",
                facecolor=card["light"],
                edgecolor=card["color"],
                linewidth=2.2,
            )
        )
        axis.add_patch(
            FancyBboxPatch(
                (x, 0.718),
                0.255,
                0.062,
                boxstyle="round,pad=0.009,rounding_size=0.015",
                facecolor=card["color"],
                edgecolor=card["color"],
                linewidth=0,
            )
        )
        figure.text(
            x + 0.018,
            0.749,
            str(card["title"]),
            fontsize=19,
            fontweight="bold",
            color="white",
            ha="left",
            va="center",
        )
        figure.text(
            x + 0.018,
            0.680,
            "优势",
            fontsize=13,
            fontweight="bold",
            color=card["color"],
            ha="left",
        )
        figure.text(
            x + 0.018,
            0.650,
            str(card["strong"]),
            fontsize=13,
            color="#273746",
            ha="left",
            va="top",
            wrap=True,
        )
        figure.text(
            x + 0.018,
            0.565,
            "典型退化",
            fontsize=13,
            fontweight="bold",
            color=card["color"],
            ha="left",
        )
        figure.text(
            x + 0.018,
            0.535,
            str(card["risk"]),
            fontsize=13,
            color="#273746",
            ha="left",
            va="top",
        )
        figure.text(
            x + 0.018,
            0.445,
            "A 基线现象",
            fontsize=13,
            fontweight="bold",
            color=card["color"],
            ha="left",
        )
        figure.text(
            x + 0.018,
            0.415,
            str(card["evidence"]),
            fontsize=12,
            color="#273746",
            ha="left",
            va="top",
        )

    chain = [
        (0.055, 0.205, "观测质量下降\n或约束不足"),
        (0.287, 0.205, "Tracking / IMU\n初始化异常"),
        (0.519, 0.205, "失跟或\nactive map reset"),
        (0.751, 0.205, "地图不连续\n指标失效"),
    ]
    for index, (x, y, text) in enumerate(chain):
        axis.add_patch(
            FancyBboxPatch(
                (x, y),
                0.19,
                0.105,
                boxstyle="round,pad=0.012,rounding_size=0.016",
                facecolor="white",
                edgecolor="#5D6D7E",
                linewidth=1.7,
            )
        )
        figure.text(
            x + 0.095,
            y + 0.052,
            text,
            fontsize=13,
            fontweight="bold",
            color="#273746",
            ha="center",
            va="center",
            linespacing=1.35,
        )
        if index < len(chain) - 1:
            axis.add_patch(
                FancyArrowPatch(
                    (x + 0.195, y + 0.052),
                    (chain[index + 1][0] - 0.005, y + 0.052),
                    arrowstyle="-|>",
                    mutation_scale=18,
                    linewidth=2,
                    color="#5D6D7E",
                )
            )

    axis.add_patch(
        FancyBboxPatch(
            (0.055, 0.085),
            0.886,
            0.075,
            boxstyle="round,pad=0.012,rounding_size=0.014",
            facecolor="#ECF0F1",
            edgecolor="#BDC3C7",
            linewidth=1,
        )
    )
    figure.text(
        0.075,
        0.122,
        "A 的边界：定位失败现象，不单独宣称因果；B 隔离视觉退化，C 隔离深度退化，D 验证 IMU 短时预测，E 设计质量感知降级与恢复。",
        fontsize=13,
        color="#34495E",
        ha="left",
        va="center",
    )

    outputs = [
        root / "presentation/A/figures/data/sensor_degradation_overview",
        root / "results/A/figures/case_bars/sensor_degradation_overview",
    ]
    for stem in outputs:
        stem.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(stem.with_suffix(".png"), dpi=220, facecolor="white")
        figure.savefig(stem.with_suffix(".svg"), facecolor="white")
    plt.close(figure)


if __name__ == "__main__":
    main()
