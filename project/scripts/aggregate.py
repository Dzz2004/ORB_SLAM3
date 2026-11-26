#!/usr/bin/env python
"""Collect metrics.json (+ metrics_final.json) of every run directory into one CSV, one row per run.

Usage:
    python scripts/aggregate.py results/C/runs [more dirs...] --out results/C/depth_ablation_runs.csv \
        [--group-out results/C/depth_ablation.csv]

Per-run columns: identification (matrix, task, base_sequence, fault, level, seed, mode, status), tracking
(coverage, lost events/frames/rate, unrecovered, recovery time, init delay, IMU-predicted frames, latency),
online ATE/RPE with segment counts (primary), final ATE/RPE with paired coverage (secondary), outage
recovery (first --fault-intervals entry), and fault_summary.json means (e.g. valid depth fraction).

--group-out writes mean/std/median/n over seeds for each (base_sequence, fault, level, mode). Statistics
of a metric use only runs where it exists; n_runs, n_ok, n_init_failed, n_crashed (ORB-SLAM3 killed by a
signal mid-sequence; status run_failed; frames before the crash are evaluated) and n_other_failed are always
given so that failures are visible next to the averages. n_ok counts only status ok, so a crashed run is never
in n_ok.
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics

ID_COLS = ["run", "matrix", "task", "base_sequence", "sequence", "fault", "level", "seed", "mode", "status",
           "exit_code", "timed_out", "wall_s"]


def g(d, *path):
    for p in path:
        if not isinstance(d, dict) or p not in d or d[p] is None:
            return None
        d = d[p]
    return d


def flatten_run(rd):
    m = json.load(open(os.path.join(rd, "metrics.json")))
    f = {}
    fp = os.path.join(rd, "metrics_final.json")
    if os.path.isfile(fp):
        f = json.load(open(fp))
    mr = m.get("matrix_run") or {}
    row = {
        "run": os.path.basename(os.path.normpath(rd)),
        "matrix": mr.get("matrix", m.get("matrix")),
        "task": mr.get("task", m.get("task")),
        "base_sequence": mr.get("base_sequence", m.get("base_sequence")),
        "sequence": m.get("sequence"),
        "fault": mr.get("fault", m.get("fault")),
        "level": mr.get("level", m.get("level")),
        "seed": m.get("seed", mr.get("seed")),
        "mode": m.get("mode", mr.get("mode")),
        "status": m.get("status"),
        "exit_code": mr.get("exit_code"),
        "timed_out": mr.get("timed_out"),
        "wall_s": mr.get("wall_s"),
        "n_frames": m.get("n_frames"),
        "initialized": m.get("initialized"),
        "init_delay_s": m.get("init_delay_s"),
        "coverage": m.get("coverage"),
        "n_valid": m.get("n_valid"),
        "lost_events": m.get("lost_events"),
        "lost_frames": m.get("lost_frames"),
        "lost_rate": m.get("lost_rate"),
        "n_unrecovered": m.get("n_unrecovered"),
        "recovery_time_mean_s": g(m, "recovery_time_s", "mean"),
        "recovery_time_median_s": g(m, "recovery_time_s", "median"),
        "n_imu_predicted": m.get("n_imu_predicted"),
        "latency_ms_mean": m.get("latency_ms_mean"),
        "alignment": m.get("alignment"),
        "segment_key": m.get("segment_key"),
        "n_segments": m.get("n_segments"),
        "n_segments_aligned": m.get("n_segments_aligned"),
        "n_valid_frames_epoch0": m.get("n_valid_frames_epoch0"),
        "n_poses_in_aligned_segments": m.get("n_poses_in_aligned_segments"),
        "aligned_coverage": m.get("aligned_coverage"),
        "ate_online_rmse": g(m, "ate", "rmse"),
        "ate_online_median": g(m, "ate", "median"),
        "rpe_trans_online_rmse": g(m, "rpe_trans", "rmse"),
        "rpe_rot_online_deg_rmse": g(m, "rpe_rot_deg", "rmse"),
        "ate_final_rmse": g(f, "ate", "rmse"),
        "rpe_trans_final_rmse": g(f, "rpe_trans", "rmse"),
        "rpe_rot_final_deg_rmse": g(f, "rpe_rot_deg", "rmse"),
        "final_scale": f.get("scale"),
        "final_n_paired": f.get("n_paired"),
        "final_paired_coverage": f.get("paired_coverage"),
    }
    # ORB-SLAM3 loop closure (run.log "Loop detected"): the final trajectory of such runs is reshaped by the
    # (inertial) loop correction, so final ATE must be read with this flag.
    log_path = os.path.join(rd, "run.log")
    if os.path.isfile(log_path):
        with open(log_path, errors="replace") as fh:
            row["n_loops"] = sum(1 for line in fh if "Loop detected" in line)
        row["loop_detected"] = row["n_loops"] > 0
    # Crash = ORB-SLAM3 killed by a signal (exit 128+N, e.g. 134 abort from Sophus NaN, 139 segfault) before the
    # end of the sequence, excluding the timeout wrapper's 124/137. Frames written before the crash are evaluated.
    ec = mr.get("exit_code")
    row["crashed"] = bool(isinstance(ec, int) and ec > 128 and ec != 137 and not mr.get("timed_out"))
    if row["crashed"]:
        row["crash_t_s"] = m.get("last_frame_t_s")
    fr = m.get("fault_recovery") or []
    if fr:
        row.update({"outage_start_s": fr[0]["fault_start_s"], "outage_end_s": fr[0]["fault_end_s"],
                    "outage_valid_frames_during": fr[0]["valid_frames_during_fault"],
                    "outage_recovered": fr[0]["recovered"], "outage_recovery_time_s": fr[0]["recovery_time_s"]})
    fs = os.path.join(rd, "fault_summary.json")
    if os.path.isfile(fs):
        for k, v in json.load(open(fs)).items():
            row["fault_" + k] = v
    if m.get("error"):
        row["error"] = str(m["error"]).splitlines()[0][:200]
    return row


def numeric(v):
    if isinstance(v, bool) or v is None or v == "":
        return None
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def group_rows(rows):
    keys = ["base_sequence", "fault", "level", "mode"]
    skip = set(ID_COLS) | {"alignment", "segment_key", "error", "initialized", "outage_recovered", "loop_detected", "crashed"}
    groups = {}
    for r in rows:
        groups.setdefault(tuple(r.get(k) for k in keys), []).append(r)
    out = []
    for key, rs in sorted(groups.items(), key=lambda kv: tuple(str(x) for x in kv[0])):
        o = dict(zip(keys, key))
        o["seeds"] = " ".join(str(r.get("seed")) for r in sorted(rs, key=lambda r: str(r.get("seed"))))
        o["n_runs"] = len(rs)
        o["n_ok"] = sum(r.get("status") == "ok" for r in rs)
        o["n_init_failed"] = sum(r.get("status") == "init_failed" for r in rs)
        o["n_crashed"] = sum(bool(r.get("crashed")) for r in rs)
        o["n_other_failed"] = o["n_runs"] - o["n_ok"] - o["n_init_failed"] - o["n_crashed"]
        o["n_loop_runs"] = sum(bool(r.get("loop_detected")) for r in rs)
        rec = [r for r in rs if r.get("outage_recovered") is not None]
        if rec:
            o["outage_n_recovered"] = sum(bool(r["outage_recovered"]) for r in rec)
        cols = []
        for r in rs:
            cols += [c for c in r if c not in skip and c not in cols]
        for c in cols:
            vals = [numeric(r.get(c)) for r in rs]
            vals = [v for v in vals if v is not None]
            if not vals:
                continue
            o[c + "_mean"] = statistics.fmean(vals)
            o[c + "_std"] = statistics.stdev(vals) if len(vals) > 1 else 0.0
            o[c + "_median"] = statistics.median(vals)
            o[c + "_n"] = len(vals)
        out.append(o)
    return out


def write_csv(path, rows, first):
    cols = list(first)
    for r in rows:
        cols += [c for c in r if c not in cols]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: ("" if r.get(c) is None else r.get(c)) for c in cols})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+", help="directories containing run directories (searched recursively)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--group-out", default=None)
    a = ap.parse_args()
    rows, bad = [], []
    for d in a.dirs:
        for mj in sorted(glob.glob(os.path.join(d, "**", "metrics.json"), recursive=True)):
            try:
                rows.append(flatten_run(os.path.dirname(mj)))
            except Exception as e:
                bad.append((mj, repr(e)))
    rows.sort(key=lambda r: tuple(str(r.get(k)) for k in ["base_sequence", "fault", "level", "mode", "seed"]))
    write_csv(a.out, rows, ID_COLS)
    st = {}
    for r in rows:
        st[r.get("status")] = st.get(r.get("status"), 0) + 1
    print(f"{len(rows)} runs -> {a.out}  status: {st}")
    if a.group_out:
        gr = group_rows(rows)
        write_csv(a.group_out, gr, ["base_sequence", "fault", "level", "mode", "seeds", "n_runs", "n_ok",
                                    "n_init_failed", "n_crashed", "n_other_failed", "n_loop_runs"])
        print(f"{len(gr)} groups -> {a.group_out}")
    for mj, e in bad:
        print(f"[unreadable] {mj}: {e}")


if __name__ == "__main__":
    main()
