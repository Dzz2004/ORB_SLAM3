#!/usr/bin/env python
"""T10 item 2: recompute sampled group means/stds from raw metrics.json and compare with the group CSV.

Read-only. The population of each group is taken from the per-run CSV written by aggregate.py
(so that runs added after the group CSV was written cannot leak in).

Semantics reproduced:
  B (results/B/visual_stress.csv, experiments/B_visual_degradation/plot_B.py):
      coverage / lost_events / n_segments / aligned_coverage / ate_online_rmse
      coverage: all runs with a numeric value (init_failed counts as 0)
      other columns: status == ok only
      error columns: additionally coverage >= 0.1 (MIN_COVERAGE_FOR_ERROR)
      std = sample stdev (n-1), 0 when n == 1
      L0 row of fault f = the clean runs (expand_clean)
  C (results/C/depth_ablation.csv, scripts/aggregate.py --group-out):
      all runs pooled; a metric contributes when its value is numeric and non-null
      std = sample stdev (n-1), 0 when n == 1
"""
import argparse
import csv
import json
import math
import os
import statistics

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

B_SAMPLE = [  # (base_sequence, fault, level, mode)
    ("office_easy_p004", "gaussian_blur", "L1", "MONOCULAR"),
    ("office_easy_p004", "gaussian_blur", "L3", "IMU_MONOCULAR"),
    ("office_easy_p004", "gaussian_blur", "L4", "IMU_RGBD"),
    ("office_easy_p004", "motion_blur", "L0", "IMU_RGBD"),
    ("office_easy_p004", "motion_blur", "L2", "MONOCULAR"),
    ("office_easy_p004", "motion_blur", "L4", "IMU_MONOCULAR"),
    ("office_easy_p004", "brightness_contrast", "L1", "IMU_RGBD"),
    ("office_easy_p004", "brightness_contrast", "L2", "IMU_MONOCULAR"),
    ("office_easy_p004", "brightness_contrast", "L4", "MONOCULAR"),
    ("office_easy_p004", "occlusion", "L1", "MONOCULAR"),
    ("office_easy_p004", "occlusion", "L3", "IMU_RGBD"),
    ("office_easy_p004", "occlusion", "L4", "IMU_MONOCULAR"),
    ("office_easy_p004", "texture_erase", "L0", "MONOCULAR"),
    ("office_easy_p004", "texture_erase", "L2", "IMU_MONOCULAR"),
    ("office_easy_p004", "texture_erase", "L4", "MONOCULAR"),
    # hard (partial B_hard in the CSV): one extra per fault where a group exists
    ("office_hard_p004", "gaussian_blur", "L0", "IMU_RGBD"),
    ("office_hard_p004", "motion_blur", "L2", "IMU_RGBD"),
    ("office_hard_p004", "brightness_contrast", "L0", "MONOCULAR"),
    ("office_hard_p004", "occlusion", "L0", "MONOCULAR"),
    ("office_hard_p004", "texture_erase", "L0", "IMU_MONOCULAR"),
]
C_SAMPLE = [
    ("office_easy_p004", "invalid_pixels", "P20", "RGBD"),
    ("office_easy_p004", "invalid_pixels", "P80", "RGBD"),
    ("office_hard_p004", "invalid_pixels", "P50", "IMU_RGBD"),
    ("office_easy_p004", "depth_noise", "N2", "IMU_RGBD"),
    ("office_easy_p004", "depth_noise", "N4", "RGBD"),
    ("office_hard_p004", "depth_noise", "N4", "RGBD"),
    ("office_easy_p004", "outage", "O05", "RGBD"),
    ("office_easy_p004", "outage", "O20", "RGBD"),
    ("office_hard_p004", "outage", "O10", "IMU_RGBD"),
    ("office_easy_p004", "depth_off", "ALL", "RGBD"),
    ("office_easy_p004", "depth_off", "ALL", "IMU_RGBD"),
    ("office_hard_p004", "depth_off", "ALL", "IMU_RGBD"),
]
B_METRICS = ["coverage", "lost_events", "n_segments", "aligned_coverage", "ate_online_rmse"]
C_METRICS = ["coverage", "lost_events", "n_segments", "aligned_coverage", "ate_online_rmse", "n_unrecovered"]


def num(v):
    if isinstance(v, bool) or v is None or v == "":
        return None
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def stats(vals):
    if not vals:
        return None, None, 0
    return statistics.fmean(vals), (statistics.stdev(vals) if len(vals) > 1 else 0.0), len(vals)


def metrics_of(runroot, name):
    p = os.path.join(ROOT, runroot, name, "metrics.json")
    return json.load(open(p)) if os.path.isfile(p) else None


def digest(m, cov_key="coverage"):
    return {
        "coverage": m.get("coverage"),
        "lost_events": m.get("lost_events"),
        "n_segments": m.get("n_segments"),
        "aligned_coverage": m.get("aligned_coverage"),
        "ate_online_rmse": (m.get("ate") or {}).get("rmse"),
        "n_unrecovered": m.get("n_unrecovered"),
        "status": m.get("status"),
    }


def check_b(problems):
    runroot = "results/B/runs"
    per = {r["run"]: r for r in csv.DictReader(open(os.path.join(ROOT, "results/B/visual_stress_runs.csv")))}
    grp = {(r["base_sequence"], r["fault"], r["level"], r["mode"]): r
           for r in csv.DictReader(open(os.path.join(ROOT, "results/B/visual_stress.csv")))}
    print("B sampled groups (per-run population: %d runs)" % len(per))
    for key in B_SAMPLE:
        seq, fault, level, mode = key
        if level == "L0":
            pop = [n for n, r in per.items() if r["base_sequence"] == seq and r["fault"] == "clean" and r["mode"] == mode]
        else:
            pop = [n for n, r in per.items() if r["base_sequence"] == seq and r["fault"] == fault
                   and r["level"] == level and r["mode"] == mode]
        ds = [digest(metrics_of(runroot, n)) for n in pop]
        if not ds or key not in grp:
            print("  SKIP %s %s %s %s: pop=%d in_group_csv=%s" % (seq, fault, level, mode, len(ds), key in grp))
            continue
        g = grp[key]
        diffs = []
        for col in B_METRICS:
            if col == "coverage":
                use = ds
            else:
                use = [d for d in ds if d["status"] == "ok"]
                if col == "ate_online_rmse":
                    use = [d for d in use if (num(d["coverage"]) or 0.0) >= 0.1]
            mean, std, n = stats([v for v in (num(d[col]) for d in use) if v is not None])
            em, es, en = num(g.get(col + "_mean")), num(g.get(col + "_std")), num(g.get(col + "_n"))
            ok = ((mean is None and em is None) or (mean is not None and em is not None and abs(mean - em) <= 1e-6)) and \
                 ((std is None and es is None) or (std is not None and es is not None and abs(std - es) <= 1e-6)) and \
                 (n == (int(en) if en is not None else 0))
            if not ok:
                diffs.append("%s: recomputed %s±%s(n%d) csv %s±%s(n%s)"
                             % (col, mean, std, n, g.get(col + "_mean"), g.get(col + "_std"), g.get(col + "_n")))
        tag = "OK " if not diffs else "MISMATCH"
        print("  %s %s %s %s %s: runs=%d %s" % (tag, seq, fault, level, mode, len(ds), "" if not diffs else "; ".join(diffs)))
        for d in diffs:
            problems.append("B %s %s %s %s %s" % (key, d))
    # group row bookkeeping: n_runs must equal population
    return


def check_c(problems):
    runroot = "results/C/runs"
    per = {r["run"]: r for r in csv.DictReader(open(os.path.join(ROOT, "results/C/depth_ablation_runs.csv")))}
    grp = {(r["base_sequence"], r["fault"], r["level"], r["mode"]): r
           for r in csv.DictReader(open(os.path.join(ROOT, "results/C/depth_ablation.csv")))}
    print("C sampled groups (per-run population: %d runs)" % len(per))
    for key in C_SAMPLE:
        seq, fault, level, mode = key
        pop = [n for n, r in per.items() if r["base_sequence"] == seq and r["fault"] == fault
               and r["level"] == level and r["mode"] == mode]
        ds = [digest(metrics_of(runroot, n)) for n in pop]
        if not ds or key not in grp:
            print("  SKIP %s %s %s %s: pop=%d in_group_csv=%s" % (seq, fault, level, mode, len(ds), key in grp))
            continue
        g = grp[key]
        diffs = []
        for col in C_METRICS:
            mean, std, n = stats([v for v in (num(d[col]) for d in ds) if v is not None])
            em, es, en = num(g.get(col + "_mean")), num(g.get(col + "_std")), num(g.get(col + "_n"))
            ok = ((mean is None and em is None) or (mean is not None and em is not None and abs(mean - em) <= 1e-6)) and \
                 ((std is None and es is None) or (std is not None and es is not None and abs(std - es) <= 1e-6)) and \
                 (n == (int(en) if en is not None else 0))
            if not ok:
                diffs.append("%s: recomputed %s±%s(n%d) csv %s±%s(n%s)"
                             % (col, mean, std, n, g.get(col + "_mean"), g.get(col + "_std"), g.get(col + "_n")))
        tag = "OK " if not diffs else "MISMATCH"
        print("  %s %s %s %s %s: runs=%d %s" % (tag, seq, fault, level, mode, len(ds), "" if not diffs else "; ".join(diffs)))
        for d in diffs:
            problems.append("C %s %s %s %s %s" % (key, d))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    import io
    import contextlib
    buf = io.StringIO()
    problems = []
    with contextlib.redirect_stdout(buf):
        check_b(problems)
        print()
        check_c(problems)
    problems.append("total sampled: B %d + C %d groups" % (len(B_SAMPLE), len(C_SAMPLE)))
    text = buf.getvalue() + "\nPROBLEMS: %d\n" % len([p for p in problems if not p.startswith("total")]) + "\n".join(problems) + "\n"
    print(text)
    if a.out:
        open(a.out, "w").write(text)


if __name__ == "__main__":
    main()
