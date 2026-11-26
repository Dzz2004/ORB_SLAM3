#!/usr/bin/env python
"""Print markdown tables for the C report / presentation, straight from the group CSV.

Usage:
    python experiments/C_depth_degradation/make_tables.py results/C/depth_ablation.csv \
        [--sequence office_easy_p004] [--fault invalid_pixels] [--decimals 3]

Every number is read from the CSV written by scripts/aggregate.py (--group-out); nothing is computed
here except mean ± std formatting. A cell whose metric exists in fewer runs than n_runs gets "n<k>/<N>".
"""
import argparse
import csv

FAULTS = ["clean", "invalid_pixels", "depth_noise", "outage", "depth_off"]
MODES = ["RGBD", "IMU_RGBD", "IMU_MONOCULAR"]
LEVELS = {"invalid_pixels": ["P00", "P20", "P50", "P80"],
          "depth_noise": ["N1", "N2", "N3", "N4"],
          "outage": ["O05", "O10", "O20"],
          "depth_off": ["ALL"]}


def f(row, col):
    v = row.get(col + "_mean")
    return float(v) if v not in (None, "") else None


def cell(row, col, d=3, scale=1.0, unit=""):
    m = f(row, col)
    if m is None:
        return "—"
    s = row.get(col + "_std")
    n = row.get(col + "_n")
    n_runs = int(row["n_runs"])
    txt = f"{m * scale:.{d}f}"
    if s not in (None, ""):
        txt += f" ± {float(s) * scale:.{d}f}"
    if n not in (None, "") and int(n) < n_runs:  # e.g. crashed run has no ATE
        txt += f" (n{int(n)}/{n_runs})"
    return txt + unit


def outcome(row):
    parts = [f"{row['n_ok']}/{row['n_runs']} ok"]
    for k, lab in [("n_init_failed", "init"), ("n_crashed", "crash"), ("n_other_failed", "other")]:
        if int(row.get(k) or 0):
            parts.append(f"{row[k]} {lab}")
    return " / ".join(parts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--sequence", default=None)
    ap.add_argument("--fault", action="append", default=None)
    ap.add_argument("--decimals", type=int, default=3)
    a = ap.parse_args()
    d = a.decimals
    rows = list(csv.DictReader(open(a.csv)))
    seqs = [a.sequence] if a.sequence else sorted({r["base_sequence"] for r in rows})
    print("| 序列 | 故障 · 档位 | 模式 | 成功 / 运行 | 在线 ATE (m) | 段数 | 对齐覆盖率 | 覆盖率 | 失跟事件 | 未恢复 | 回环运行 |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for seq in seqs:
        for fault in [x for x in FAULTS if not a.fault or x in a.fault]:
            levels = ["clean"] if fault == "clean" else LEVELS[fault]
            for lv in levels:
                for mode in MODES:
                    hit = [r for r in rows if r["base_sequence"] == seq and r["fault"] == fault
                           and r["level"] == lv and r["mode"] == mode]
                    for r in hit:
                        lab = "无扰动" if fault == "clean" else f"{fault} {lv}"
                        print(f"| {seq} | {lab} | {mode} | {outcome(r)} | {cell(r, 'ate_online_rmse', d)} | "
                              f"{cell(r, 'n_segments', 1)} | {cell(r, 'aligned_coverage', 3)} | "
                              f"{cell(r, 'coverage', 3)} | {cell(r, 'lost_events', 1)} | "
                              f"{cell(r, 'n_unrecovered', 1)} | {r.get('n_loop_runs')} |")


if __name__ == "__main__":
    main()
