#!/usr/bin/env python
"""T10 item 2 (full sweep): recompute every group row of the two group CSVs from raw metrics.json.

Read-only. Population and semantics are the same as check_groups.py:
  B: per-run CSV population; clean runs are the L0 row of every fault; coverage uses all runs,
     other columns status==ok; ATE/RPE also require coverage >= 0.1; sample stdev.
  C: all runs pooled; numeric non-null values only; sample stdev.
Compared fields: coverage, lost_events, n_segments, aligned_coverage, ate_online_rmse (mean/std/n).
Also checks n_runs of each row.
"""
import csv
import json
import math
import os
import statistics

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def num(v):
    if v in (None, ""):
        return None
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def stats(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, None, 0
    return statistics.fmean(vals), (statistics.stdev(vals) if len(vals) > 1 else 0.0), len(vals)


def metric(name, runroot):
    return json.load(open(os.path.join(ROOT, runroot, name, "metrics.json")))


def get(m, path):
    d = m
    for p in path:
        d = (d or {}).get(p)
    return num(d)


def cmp(problems, tag, key, col, got, want):
    mean, sd, n = got
    em, es, en = want
    ok = ((mean is None and em is None) or (mean is not None and em is not None and abs(mean - em) <= 1e-6)) and \
         ((sd is None and es is None) or (sd is not None and es is not None and abs(sd - es) <= 1e-6)) and \
         (n == (int(en) if en is not None else 0))
    if not ok:
        problems.append("%s %s %s: recomputed %s±%s(n%d) csv %s±%s(n%s)"
                        % (tag, key, col, mean, sd, n, em, es, en))


def check_b(problems):
    runroot = "results/B/runs"
    per = {r["run"]: r for r in csv.DictReader(open(os.path.join(ROOT, "results/B/visual_stress_runs.csv")))}
    clean_runs = {}
    for n, r in per.items():
        clean_runs.setdefault((r["base_sequence"], r["fault"], r["mode"]), []).append(n)
    groups = list(csv.DictReader(open(os.path.join(ROOT, "results/B/visual_stress.csv"))))
    checked = 0
    for g in groups:
        key = (g["base_sequence"], g["fault"], g["level"], g["mode"])
        if g["level"] == "L0":
            pop = clean_runs.get((key[0], "clean", key[3]), [])
        else:
            pop = [n for n, r in per.items()
                   if (r["base_sequence"], r["fault"], r["level"], r["mode"]) == key]
        ms = [metric(n, runroot) for n in pop]
        if len(ms) != int(g["n_runs"]):
            problems.append("B %s: population %d != csv n_runs %d" % (key, len(ms), int(g["n_runs"])))
        for col, path, covfilter, okonly in [
                ("coverage", ("coverage",), False, False),
                ("lost_events", ("lost_events",), False, True),
                ("n_segments", ("n_segments",), False, True),
                ("aligned_coverage", ("aligned_coverage",), False, True),
                ("ate_online_rmse", ("ate", "rmse"), True, True)]:
            vals = []
            for m in ms:
                if okonly and m.get("status") != "ok":
                    continue
                if covfilter and (num(m.get("coverage")) or 0.0) < 0.1:
                    continue
                vals.append(get(m, path))
            cmp(problems, "B", key, col, stats(vals),
                (num(g.get(col + "_mean")), num(g.get(col + "_std")), num(g.get(col + "_n"))))
            checked += 1
    return len(groups), checked


def check_c(problems):
    runroot = "results/C/runs"
    per = {r["run"]: r for r in csv.DictReader(open(os.path.join(ROOT, "results/C/depth_ablation_runs.csv")))}
    groups = list(csv.DictReader(open(os.path.join(ROOT, "results/C/depth_ablation.csv"))))
    checked = 0
    for g in groups:
        key = (g["base_sequence"], g["fault"], g["level"], g["mode"])
        pop = [n for n, r in per.items()
               if (r["base_sequence"], r["fault"], r["level"], r["mode"]) == key]
        ms = [metric(n, runroot) for n in pop]
        if len(ms) != int(g["n_runs"]):
            problems.append("C %s: population %d != csv n_runs %d" % (key, len(ms), int(g["n_runs"])))
        for col, path in [("coverage", ("coverage",)), ("lost_events", ("lost_events",)),
                          ("n_segments", ("n_segments",)), ("aligned_coverage", ("aligned_coverage",)),
                          ("ate_online_rmse", ("ate", "rmse"))]:
            vals = [get(m, path) for m in ms]
            cmp(problems, "C", key, col, stats(vals),
                (num(g.get(col + "_mean")), num(g.get(col + "_std")), num(g.get(col + "_n"))))
            checked += 1
    return len(groups), checked


def main():
    problems = []
    bg, bc = check_b(problems)
    cg, cc = check_c(problems)
    text = "B: %d groups, %d values; C: %d groups, %d values; mismatches: %d\n" % (
        bg, bc, cg, cc, len(problems))
    for p in problems:
        text += "MISMATCH " + p + "\n"
    print(text, end="")
    out = os.path.join(ROOT, "results/T7_audit/group_check_all.txt")
    open(out, "w").write(text)


if __name__ == "__main__":
    main()
