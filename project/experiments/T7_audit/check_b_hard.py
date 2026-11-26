#!/usr/bin/env python
"""T10 follow-up: verify the B hard (partial matrix) content of presentation/B.

Checks:
  1. every cell of 实现说明.md 5.6 hard coverage table vs results/B/visual_stress.csv
     (mean/std/n; "0（3 次均未初始化）" = mean 0 and all runs failed init; "—" = no group row)
  2. hard L0 baseline coverage 0.450/0.302/0.624 and L0 segments/lost 86.7/98.7/154.3,
     20.0/96.0/152.3
  3. completed-setting list (11 settings) and the 132 / 36 / 96 run-count bookkeeping
  4. the 6 init_failed runs are exactly gaussian_blur L4 MONOCULAR/IMU_MONOCULAR
  5. hard figures 08/09/10 exist; slides page 7 hard numbers present and consistent
Read-only.
"""
import csv
import os
import re
import statistics

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DOC = os.path.join(ROOT, "presentation/B/实现说明.md")
SLIDES = os.path.join(ROOT, "presentation/B/slides.md")
MODES = ["MONOCULAR", "IMU_MONOCULAR", "IMU_RGBD"]
LEVELS = ["L1", "L2", "L3", "L4"]


def load_groups():
    rows = [r for r in csv.DictReader(open(os.path.join(ROOT, "results/B/visual_stress.csv")))
            if r["base_sequence"] == "office_hard_p004"]
    return {(r["fault"], r["level"], r["mode"]): r for r in rows}


def load_runs():
    return [r for r in csv.DictReader(open(os.path.join(ROOT, "results/B/visual_stress_runs.csv")))
            if r["base_sequence"] == "office_hard_p004"]


def parse_table():
    text = open(DOC).read()
    sec = text.split("### 5.6", 1)[1].split("## 6.", 1)[0]
    cells = {}
    for line in sec.splitlines():
        if not line.startswith("|"):
            continue
        parts = [p.strip() for p in line.strip().strip("|").split("|")]
        if len(parts) != 6 or parts[0] in ("故障", "---") or parts[1].startswith("-"):
            continue
        fault, mode = parts[0], parts[1]
        for lvl, cell in zip(LEVELS, parts[2:]):
            cells[(fault, lvl, mode)] = cell
    return cells, sec


def num(s):
    return float(s)


def main():
    bad, n = [], 0

    def chk(label, cond, got, want):
        nonlocal n
        n += 1
        if not cond:
            bad.append("%s: got=%s expected=%s" % (label, got, want))

    groups = load_groups()
    runs = load_runs()
    cells, sec = parse_table()
    faults = ["gaussian_blur", "motion_blur", "brightness_contrast", "occlusion", "texture_erase"]
    chk("table 15 rows x 4 levels", len(cells) == 60, len(cells), 60)
    for f in faults:
        for m in MODES:
            chk("table row present %s %s" % (f, m), all((f, l, m) in cells for l in LEVELS), None, None)

    # 1. cells
    for (f, l, m), cell in sorted(cells.items()):
        g = groups.get((f, l, m))
        if cell == "—":
            chk("range %s %s %s = no row" % (f, l, m), g is None, g is not None, None)
            continue
        chk("cell %s %s %s exists" % (f, l, m), g is not None, g is not None, None)
        if g is None:
            continue
        if cell.startswith("0（"):
            chk("zero cell %s %s %s mean 0" % (f, l, m), num(g["coverage_mean"]) == 0.0,
                g["coverage_mean"], 0.0)
            chk("zero cell %s %s %s all init_failed" % (f, l, m),
                int(g["n_ok"]) == 0 and int(g["n_init_failed"]) == int(g["n_runs"]),
                (g["n_ok"], g["n_init_failed"], g["n_runs"]), "all init_failed")
            chk("zero cell %s %s %s 3 runs" % (f, l, m), int(g["n_runs"]) == 3, g["n_runs"], 3)
            continue
        mt = re.match(r"^([0-9.]+)(?:±([0-9.]+))?（n=(\d+)）$", cell)
        chk("cell format %s %s %s" % (f, l, m), mt is not None, cell, "mean[±std]（n=k）")
        if not mt:
            continue
        mean, sd, k = num(mt.group(1)), (num(mt.group(2)) if mt.group(2) else None), int(mt.group(3))
        chk("cell %s %s %s mean" % (f, l, m), abs(num(g["coverage_mean"]) - mean) <= 5.1e-4,
            g["coverage_mean"], mean)
        chk("cell %s %s %s n" % (f, l, m), int(g["coverage_n"]) == k, g["coverage_n"], k)
        if sd is not None:
            chk("cell %s %s %s std" % (f, l, m),
                abs(num(g["coverage_std"]) - sd) <= 5.1e-4, g["coverage_std"], sd)
            chk("cell %s %s %s n_runs" % (f, l, m), int(g["n_runs"]) == k, g["n_runs"], k)

    # 2. baseline and L0 segments/lost
    for m, cov, segs, lost in [("MONOCULAR", 0.450, 86.7, 20.0),
                               ("IMU_MONOCULAR", 0.302, 98.7, 96.0),
                               ("IMU_RGBD", 0.624, 154.3, 152.3)]:
        g = groups[("gaussian_blur", "L0", m)]
        chk("L0 %s cov" % m, "%.3f" % num(g["coverage_mean"]) == "%.3f" % cov,
            g["coverage_mean"], cov)
        chk("L0 %s segs" % m, "%.1f" % num(g["n_segments_mean"]) == "%.1f" % segs,
            g["n_segments_mean"], segs)
        chk("L0 %s lost" % m, "%.1f" % num(g["lost_events_mean"]) == "%.1f" % lost,
            g["lost_events_mean"], lost)
        chk("L0 %s n=3" % m, int(g["n_runs"]) == 3, g["n_runs"], 3)

    # 3. completed settings and run-count bookkeeping
    done = sorted({(r["fault"], r["level"]) for r in runs
                   if r["fault"] != "clean" and r["level"] != "L0"})
    want_done = sorted([("gaussian_blur", "L%d" % i) for i in range(1, 5)] +
                       [("motion_blur", "L%d" % i) for i in range(1, 5)] +
                       [("brightness_contrast", "L%d" % i) for i in range(1, 3)] +
                       [("occlusion", "L2")])
    chk("11 completed settings", done == want_done and len(done) == 11, done, want_done)
    total_counts = sum(int(g["n_runs"]) for g in groups.values())
    chk("132 group-run counts", total_counts == 132, total_counts, 132)
    l0_counts = sum(int(g["n_runs"]) for k, g in groups.items() if k[1] == "L0")
    l0_unique = sum(1 for r in runs if r["fault"] == "clean")
    chk("45 L0 counts", l0_counts == 45, l0_counts, 45)
    chk("9 unique clean runs", l0_unique == 9, l0_unique, 9)
    chk("36 duplicated L0 counts", l0_counts - l0_unique == 36, l0_counts - l0_unique, 36)
    chk("unique runs 96", len(runs) == 96, len(runs), 96)
    chk("132-36=96", total_counts - (l0_counts - l0_unique) == len(runs),
        total_counts - (l0_counts - l0_unique), len(runs))

    # 4. init_failed runs
    inits = [(r["fault"], r["level"], r["mode"], r["seed"]) for r in runs
             if r["status"] == "init_failed"]
    chk("6 init_failed", len(inits) == 6, len(inits), 6)
    chk("init_failed set", sorted((f, l, m) for f, l, m, s in inits) ==
        sorted([("gaussian_blur", "L4", "MONOCULAR")] * 3 +
               [("gaussian_blur", "L4", "IMU_MONOCULAR")] * 3), inits, "gaussian L4 mono x2 modes x3 seeds")
    chk("10 ok + 6 init + 80? total", sum(1 for r in runs if r["status"] == "ok") == 90,
        sum(1 for r in runs if r["status"] == "ok"), 90)
    chk("other_failed 0", sum(1 for r in runs if r["status"] not in ("ok", "init_failed")) == 0,
        sum(1 for r in runs if r["status"] not in ("ok", "init_failed")), 0)

    # 5. narrative values in 5.6 and slides
    for sub in ["0.401±0.041", "0.264±0.028", "0.030±0.007", "0.241±0.006", "0.607±0.001",
                "0.624 降到 gaussian L4 的 0.030", "motion L4 的 0.495",
                "0.006 / 0.016", "L1：0.422 / 0.289 / 0.625",
                "（0.265 / 0.193 / 0.571）", "86.7、IMU_MONOCULAR 98.7、IMU_RGBD 154.3",
                "20.0 / 96.0 / 152.3", "132 个组内运行数里有 36 个", "唯一运行数是 96"]:
        chk("5.6 text %s" % sub, sub in sec, None, sub)
    sl = open(SLIDES).read()
    for sub in ["96/189（90 ok + 6 init_failed）", "0.450 / 0.302 / 0.624",
                "86.7 / 98.7 / 154.3", "6 次 init_failed 都是 gaussian L4"]:
        chk("slides %s" % sub, sub in sl, None, sub)
    for fig in ["08_coverage_vs_level_hard.png", "09_lost_events_vs_level_hard.png",
                "10_segments_vs_level_hard.png"]:
        p = os.path.join(ROOT, "presentation/B/figures", fig)
        chk("fig %s" % fig, os.path.isfile(p) and os.path.getsize(p) > 0, os.path.isfile(p), True)

    text = "B hard: checked %d items, %d mismatches\n" % (n, len(bad))
    for x in bad:
        text += "MISMATCH " + x + "\n"
    open(os.path.join(ROOT, "results/T7_audit/b_hard_check.txt"), "w").write(text)
    print(text, end="")


if __name__ == "__main__":
    main()
