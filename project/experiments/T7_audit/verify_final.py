#!/usr/bin/env python
"""T10 final check: evaluation caliber, per-run CSV vs metrics.json, anomalous runs.

Read-only. Usage:
    TMPDIR=$T env/bin/python -I experiments/T7_audit/verify_final.py --out results/T7_audit/final_check.txt \
        [--csv results/B/visual_stress_runs.csv ...]

Checks
1. Every run dir with metrics.json uses the final caliber: drop_segment_first_frames == 1,
   min_tracked_points present, segment_key == "map_id,frame_epoch", n_ok_but_empty present,
   loop_closure present. Runs without metrics.json are listed with their matrix_run.json status.
2. For every per-run CSV given, each row is compared to the run's metrics.json on the numeric fields
   aggregate.py exports (coverage, n_segments, ate_online_rmse, ...). Exact match expected.
   Every run with metrics.json must have exactly one CSV row, and vice versa.
3. Anomalous runs: non-ok status, n_ok_but_empty > 0, ate_online_rmse > 1 m, crashed, loop runs whose
   final ATE > 1 m, init_failed runs.
4. Seed spread: groups (base_sequence, fault, level, mode) with >= 2 ok seeds whose ATE max/min > 3
   or coverage range > 0.2.
"""
import argparse
import csv
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUN_ROOTS = [os.path.join(ROOT, "results", "B", "runs"), os.path.join(ROOT, "results", "C", "runs")]

# Fields aggregate.py copies 1:1 from metrics.json / metrics_final.json.
FIELDS = [
    ("coverage", lambda m, f: m.get("coverage")),
    ("n_valid", lambda m, f: m.get("n_valid")),
    ("n_segments", lambda m, f: m.get("n_segments")),
    ("n_segments_aligned", lambda m, f: m.get("n_segments_aligned")),
    ("n_poses_in_aligned_segments", lambda m, f: m.get("n_poses_in_aligned_segments")),
    ("aligned_coverage", lambda m, f: m.get("aligned_coverage")),
    ("ate_online_rmse", lambda m, f: (m.get("ate") or {}).get("rmse")),
    ("ate_online_median", lambda m, f: (m.get("ate") or {}).get("median")),
    ("rpe_trans_online_rmse", lambda m, f: (m.get("rpe_trans") or {}).get("rmse")),
    ("rpe_rot_online_deg_rmse", lambda m, f: (m.get("rpe_rot_deg") or {}).get("rmse")),
    ("ate_final_rmse", lambda m, f: (f.get("ate") or {}).get("rmse")),
    ("final_paired_coverage", lambda m, f: f.get("paired_coverage")),
    ("n_frames", lambda m, f: m.get("n_frames")),
    ("status", lambda m, f: m.get("status")),
]


def num(v):
    if isinstance(v, bool) or v is None or v == "":
        return None
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def close(a, b):
    if a is None and b is None:
        return True
    if a is None or b is None:
        return False
    return abs(a - b) <= 1e-9 * max(1.0, abs(a), abs(b))


def load_run(rd):
    mp = os.path.join(rd, "metrics.json")
    if not os.path.isfile(mp):
        return None
    m = json.load(open(mp))
    fp = os.path.join(rd, "metrics_final.json")
    f = json.load(open(fp)) if os.path.isfile(fp) else {}
    return m, f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--csv", action="append", default=[], help="per-run CSV from aggregate.py")
    a = ap.parse_args()
    out = []
    P = out.append

    runs = {}
    for root in RUN_ROOTS:
        if not os.path.isdir(root):
            continue
        tag = os.path.relpath(root, ROOT)
        for name in sorted(os.listdir(root)):
            rd = os.path.join(root, name)
            if not os.path.isdir(rd) or name.startswith("_"):
                continue
            runs[(tag, name)] = rd

    # ---- 1. caliber ----
    P("== 1. 评测口径 (each metrics.json must be final caliber)")
    missing, stale, n_all, status_only, noseg = [], [], 0, [], []
    for (tag, name), rd in sorted(runs.items()):
        lr = load_run(rd)
        if lr is None:
            mr = os.path.join(rd, "matrix_run.json")
            st = None
            if os.path.isfile(mr):
                info = json.load(open(mr))
                st = "exit=%s timed_out=%s" % (info.get("exit_code"), info.get("timed_out"))
            missing.append("%s/%s (%s)" % (tag, name, st))
            continue
        m, f = lr
        n_all += 1
        # A run with fewer than 3 paired poses gets no segment statistics at all (evaluate.py early
        # return, "note" set); an init_failed / failed run gets a status-only metrics.json. Neither is
        # a stale caliber, but both are listed separately.
        if m.get("n_frames") is None:
            status_only.append("%s/%s status=%s" % (tag, name, m.get("status")))
            continue
        if m.get("n_paired") is not None and m["n_paired"] < 3:
            noseg.append("%s/%s status=%s n_paired=%s" % (tag, name, m.get("status"), m["n_paired"]))
            continue
        problems = []
        if m.get("drop_segment_first_frames") != 1:
            problems.append("drop_segment_first_frames=%r" % (m.get("drop_segment_first_frames"),))
        if "n_dropped_segment_first" not in m:
            problems.append("no n_dropped_segment_first")
        if m.get("min_tracked_points") is None:
            problems.append("no min_tracked_points")
        if m.get("segment_key") != "map_id,frame_epoch":
            problems.append("segment_key=%r" % (m.get("segment_key"),))
        if "n_ok_but_empty" not in m:
            problems.append("no n_ok_but_empty")
        if "loop_closure" not in m:
            problems.append("no loop_closure")
        if problems:
            stale.append("%s/%s status=%s: %s" % (tag, name, m.get("status"), "; ".join(problems)))
    P("runs with metrics.json: %d   final caliber: %d   stale: %d   status-only: %d   <3 poses: %d   missing: %d"
      % (n_all, n_all - len(stale) - len(status_only) - len(noseg), len(stale),
         len(status_only), len(noseg), len(missing)))
    for s in status_only:
        P("  STATUS-ONLY " + s)
    for s in noseg:
        P("  NO-SEGMENT " + s)
    for s in stale:
        P("  STALE " + s)
    for s in missing:
        P("  NO-METRICS " + s)

    # ---- 2. CSV vs metrics ----
    P("")
    P("== 2. per-run CSV vs metrics.json")
    if not a.csv:
        P("(no --csv given)")
    for cp in a.csv:
        if not os.path.isfile(cp):
            P("MISSING CSV: %s" % cp)
            continue
        rows = list(csv.DictReader(open(cp)))
        byname = {}
        dup = []
        for r in rows:
            if r["run"] in byname:
                dup.append(r["run"])
            byname[r["run"]] = r
        P("%s: %d rows, %d runs" % (os.path.relpath(cp, ROOT), len(rows), len(byname)))
        if dup:
            P("  DUPLICATE rows: %d (%s...)" % (len(dup), ", ".join(sorted(set(dup))[:5])))
        # The CSV belongs to one run root; match rows against that root only.
        rel = os.path.relpath(cp, ROOT)
        csv_tag = "results/B/runs" if rel.startswith(os.path.join("results", "B")) else (
            "results/C/runs" if rel.startswith(os.path.join("results", "C")) else None)
        mism, absent = [], 0
        checked = 0
        for (tag, name), rd in sorted(runs.items()):
            if csv_tag is not None and tag != csv_tag:
                continue
            lr = load_run(rd)
            if lr is None:
                continue
            r = byname.get(name)
            if r is None:
                absent += 1
                continue
            m, f = lr
            checked += 1
            bad = []
            for col, getter in FIELDS:
                if col not in r:
                    continue
                exp = getter(m, f)
                got = r[col]
                if col == "status":
                    if (got or None) != (exp or None):
                        bad.append("%s: csv=%r metrics=%r" % (col, got, exp))
                    continue
                if not close(num(got), num(exp)):
                    bad.append("%s: csv=%r metrics=%r" % (col, got, exp))
            if bad:
                mism.append("%s: %s" % (name, "; ".join(bad)))
        # rows in CSV with no metrics.json
        orphan = [n for n in byname if (csv_tag, n) not in runs]
        P("  checked vs metrics: %d   mismatched: %d   run dirs not in CSV: %d   CSV rows without a run dir: %d"
          % (checked, len(mism), absent, len(orphan)))
        for s in mism[:40]:
            P("  MISMATCH " + s)
        if len(mism) > 40:
            P("  ... %d more" % (len(mism) - 40))
        for s in orphan[:10]:
            P("  ORPHAN-ROW " + s)

    # ---- 3. anomalous runs ----
    P("")
    P("== 3. anomalous runs")
    status_counts = {}
    anomalies = []
    for (tag, name), rd in sorted(runs.items()):
        lr = load_run(rd)
        if lr is None:
            anomalies.append("%s/%s: no metrics.json (see section 1)" % (tag, name))
            continue
        m, f = lr
        st = m.get("status")
        status_counts[st] = status_counts.get(st, 0) + 1
        notes = []
        if st != "ok":
            notes.append("status=%s" % st)
        if (m.get("n_ok_but_empty") or 0) > 0:
            notes.append("n_ok_but_empty=%d" % m["n_ok_but_empty"])
        ate = (m.get("ate") or {}).get("rmse")
        if ate is not None and ate > 1.0:
            notes.append("ate_online=%.3f" % ate)
        if (m.get("n_unrecovered") or 0) > 0:
            notes.append("n_unrecovered=%d" % m["n_unrecovered"])
        if m.get("loop_closure") and (f.get("ate") or {}).get("rmse", 0) and (f.get("ate") or {}).get("rmse") > 1.0:
            notes.append("loop_final_ate=%.2f" % f["ate"]["rmse"])
        if notes:
            anomalies.append("%s/%s: %s" % (tag, name, ", ".join(notes)))
    P("status counts: %s" % (dict(sorted(status_counts.items(), key=lambda kv: str(kv[0]))),))
    for s in anomalies:
        P("  " + s)

    # ---- 4. seed spread ----
    P("")
    P("== 4. seed spread (groups with >=2 ok seeds, ate ratio>3 or coverage range>0.2)")
    groups = {}
    for (tag, name), rd in sorted(runs.items()):
        lr = load_run(rd)
        if lr is None:
            continue
        m, f = lr
        if m.get("status") != "ok":
            continue
        key = (tag, m.get("base_sequence") or m.get("sequence"), m.get("fault"), m.get("level"), m.get("mode"))
        groups.setdefault(key, []).append(m)
    n_group = 0
    for key, ms in sorted(groups.items(), key=lambda kv: tuple(str(x) for x in kv[0])):
        if len(ms) < 2:
            continue
        ates = [(m.get("ate") or {}).get("rmse") for m in ms]
        covs = [m.get("coverage") for m in ms]
        ates = [x for x in ates if x is not None]
        covs = [x for x in covs if x is not None]
        ratio = (max(ates) / min(ates)) if len(ates) > 1 and min(ates) > 0 else None
        rng = (max(covs) - min(covs)) if len(covs) > 1 else None
        if (ratio is not None and ratio > 3) or (rng is not None and rng > 0.2):
            n_group += 1
            P("  %s %s %s %s %s: seeds=%s cov %.3f-%.3f ate %.4f-%.4f segs %s"
              % (key[0], key[1], key[2], key[3], key[4],
                 " ".join(str(m.get("seed")) for m in ms),
                 min(covs) if covs else float("nan"), max(covs) if covs else float("nan"),
                 min(ates) if ates else float("nan"), max(ates) if ates else float("nan"),
                 "%s-%s" % (min(m.get("n_segments") or 0 for m in ms), max(m.get("n_segments") or 0 for m in ms))))
    P("groups flagged: %d" % n_group)

    text = "\n".join(out)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as fh:
        fh.write(text + "\n")
    print(text)


if __name__ == "__main__":
    sys.exit(main())
