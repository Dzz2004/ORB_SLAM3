#!/usr/bin/env python
"""T10 item 3 follow-up: verify the revised C outage figures and the new 6.4 numbers.

Read-only. Three checks:
  1. outage-window valid fraction recomputed from each run's tracking.csv
     (valid == 1 and quality_rgb >= 1, window [outage_start_s, outage_end_s))
     vs depth_ablation_runs.csv outage_valid_frames_during, and vs the group values
     quoted in presentation/C/实现说明.md 6.4.
  2. lost-event start times for the easy RGBD outage runs recomputed from tracking.csv,
     vs the times quoted in 6.4 (distinct values; all after the window).
  3. presentation/C/figures/04_outage_easy.png and 08_outage_hard.png are byte-identical
     to the regenerated results/C/figures/03_outage_office_{easy,hard}_p004.png.
"""
import csv
import hashlib
import os
import statistics

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CLAIM_EASY = 1.0
CLAIM_HARD = {
    ("RGBD", "O05"): 1.0,
    ("RGBD", "O10"): 1.0,
    ("RGBD", "O20"): 0.8333333333333334,
    ("IMU_RGBD", "O05"): 0.6,
    ("IMU_RGBD", "O10"): 0.5666666666666667,
    ("IMU_RGBD", "O20"): 0.11666666666666667,
}
CLAIM_HARD_STD = {
    ("RGBD", "O20"): 0.28867513459481287,
    ("IMU_RGBD", "O05"): 0.5291502622129182,
    ("IMU_RGBD", "O10"): 0.11547005383792516,
    ("IMU_RGBD", "O20"): 0.2020725942163676,
}
CLAIM_HARD_SEEDS = {
    ("RGBD", "O20"): [0.5, 1.0, 1.0],
    ("IMU_RGBD", "O05"): [0.0, 0.8, 1.0],
    ("IMU_RGBD", "O10"): [0.7, 0.5, 0.5],
    ("IMU_RGBD", "O20"): [0.35, 0.0, 0.0],
}
CLAIM_STARTS = [28.0, 28.3, 28.7, 63.7, 65.2, 67.3, 75.7, 75.8, 76.3, 85.6]


def load(run):
    p = os.path.join(ROOT, "results/C/runs", run, "tracking.csv")
    rows = [r for r in csv.DictReader(open(p)) if None not in r.values()]
    t = [float(r["timestamp"]) for r in rows]
    v = [(r["valid"] == "1" and float(r["quality_rgb"] or 0) >= 1) for r in rows]
    return t, v


def window(t, v, t0, t1):
    n = nv = 0
    for ti, vi in zip(t, v):
        if t0 <= ti < t1:
            n += 1
            nv += int(vi)
    return n, nv


def lost_starts(t, v):
    fv = next((i for i, x in enumerate(v) if x), None)
    out, i = [], fv
    while i is not None and i < len(v):
        if not v[i]:
            j = i
            while j < len(v) and not v[j]:
                j += 1
            out.append(t[i])
            i = j
        else:
            i += 1
    return out


def sha(p):
    return hashlib.sha256(open(os.path.join(ROOT, p), "rb").read()).hexdigest()


def main():
    bad, n = [], 0

    def chk(label, cond, got, want):
        nonlocal n
        n += 1
        if not cond:
            bad.append("%s: got=%s expected=%s" % (label, got, want))

    runs = [r for r in csv.DictReader(open(os.path.join(ROOT, "results/C/depth_ablation_runs.csv")))
            if r["fault"] == "outage"]
    chk("36 outage runs", len(runs) == 36, len(runs), 36)

    per_group = {}
    for r in runs:
        t, v = load(r["run"])
        t0, t1 = float(r["outage_start_s"]), float(r["outage_end_s"])
        wn, wv = window(t, v, t0, t1)
        chk(r["run"] + " window frames", wn > 0, wn, ">0")
        chk(r["run"] + " during vs tracking",
            wv == int(r["outage_valid_frames_during"]),
            wv, r["outage_valid_frames_during"])
        frac = wv / wn if wn else None
        per_group.setdefault((r["base_sequence"], r["mode"], r["level"]), []).append((int(r["seed"]), frac))
        if r["base_sequence"] == "office_hard_p004" and r["mode"] == "RGBD" and r["level"] in ("O05", "O10"):
            chk(r["run"] + " frac 1.0", frac == 1.0, frac, 1.0)

    # easy: every run 1.000
    easy = [x for k, v in per_group.items() if k[0] == "office_easy_p004" for x in v]
    chk("easy 18/18 = 1.000", len(easy) == 18 and all(f == CLAIM_EASY for _, f in easy),
        [f for _, f in easy], CLAIM_EASY)

    # hard group means/std/seed values
    for key, want in CLAIM_HARD.items():
        vals = [f for _, f in per_group[("office_hard_p004",) + key]]
        mean = statistics.fmean(vals)
        chk("hard %s %s mean" % key, abs(mean - want) <= 1e-9, mean, want)
        if key in CLAIM_HARD_STD:
            sd = statistics.stdev(vals)
            chk("hard %s %s std" % key, abs(sd - CLAIM_HARD_STD[key]) <= 1e-9, sd, CLAIM_HARD_STD[key])
        if key in CLAIM_HARD_SEEDS:
            chk("hard %s %s seeds" % key,
                [round(f, 6) for f in vals] == CLAIM_HARD_SEEDS[key],
                [round(f, 6) for f in vals], CLAIM_HARD_SEEDS[key])
        print("hard %-8s %-4s per-seed %s mean %.4f std %.4f"
              % (key[0], key[1], [round(f, 3) for f in vals], mean, statistics.stdev(vals)))

    # easy RGBD lost-event starts
    easy_rgbd = [r for r in runs if r["base_sequence"] == "office_easy_p004" and r["mode"] == "RGBD"]
    starts = []
    for r in easy_rgbd:
        t, v = load(r["run"])
        ev = lost_starts(t, v)
        chk(r["run"] + " lost events", len(ev) == int(r["lost_events"]), len(ev), r["lost_events"])
        starts.extend(ev)
    uniq = sorted({round(x, 1) for x in starts})
    chk("easy RGBD distinct starts", uniq == CLAIM_STARTS, uniq, CLAIM_STARTS)
    chk("all starts after window (<=12 s)", all(x > 12.0 for x in starts), max(starts), ">12")
    chk("earliest start 28.0", abs(min(starts) - 28.0) < 1e-9, min(starts), 28.0)

    # figure copies
    pairs = [("presentation/C/figures/04_outage_easy.png", "results/C/figures/03_outage_office_easy_p004.png"),
             ("presentation/C/figures/08_outage_hard.png", "results/C/figures/03_outage_office_hard_p004.png")]
    for a, b in pairs:
        chk(a + " identical to source", sha(a) == sha(b), sha(a)[:12], sha(b)[:12])

    text = "checked %d items, %d mismatches\n" % (n, len(bad))
    for x in bad:
        text += "MISMATCH " + x + "\n"
    open(os.path.join(ROOT, "results/T7_audit/c_outage_check.txt"), "w").write(text)
    print(text, end="")


if __name__ == "__main__":
    main()
