"""A baseline table from the clean (no-fault) runs of the B and C matrices (read-only).

Scans RUN_ROOTS (default results/B/runs results/C/runs) for <seq>__clean_s<seed>__<MODE> run directories with
metrics.json (online trajectory, segmented) and metrics_final.json, and writes
  results/A/baseline_runs.csv     one row per run
  results/A/baseline_metrics.csv  one row per (sequence, mode, source): mean/std/min/max over seeds
Missing / failed runs are listed with status, never filled with 0.
Usage: python collect_baseline.py [--roots DIR ...] [--out results/A]
"""
import argparse, csv, glob, json, os, re
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
NAME = re.compile(r"^(?P<seq>.+)__clean_s(?P<seed>\d+)__(?P<mode>MONOCULAR|IMU_MONOCULAR|RGBD|IMU_RGBD)$")
MODES = ["MONOCULAR", "IMU_MONOCULAR", "RGBD", "IMU_RGBD"]


def g(d, *path):
    for p in path:
        if not isinstance(d, dict) or d.get(p) is None:
            return None
        d = d[p]
    return d


def load(path):
    try:
        return json.load(open(path))
    except (OSError, ValueError):
        return None


def latency(run):
    try:
        v = np.array([float(r["latency_ms"]) for r in csv.DictReader(open(os.path.join(run, "tracking.csv")))])
        return float(v.mean()), float(np.percentile(v, 95))
    except (OSError, KeyError, ValueError):
        return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--roots", nargs="+", default=["results/B/runs", "results/C/runs"])
    ap.add_argument("--out", default="results/A")
    a = ap.parse_args()
    rows = []
    for root in a.roots:
        src = os.path.basename(os.path.dirname(os.path.join(ROOT, root).rstrip("/")))  # B or C
        for run in sorted(glob.glob(os.path.join(ROOT, root, "*__clean_s*__*"))):
            m = NAME.match(os.path.basename(run))
            if not m:
                continue
            on, fi = load(os.path.join(run, "metrics.json")), load(os.path.join(run, "metrics_final.json"))
            lm, lp = latency(run)
            rows.append({
                "source": src, "sequence": m["seq"], "mode": m["mode"], "seed": int(m["seed"]),
                "run_dir": os.path.relpath(run, ROOT),
                "status": (on or {}).get("status", "missing_metrics" if on is None else "ok"),
                "alignment": g(on, "alignment"), "segment_key": g(on, "segment_key"),
                "coverage": g(on, "coverage"), "init_delay_s": g(on, "init_delay_s"),
                "n_segments": g(on, "n_segments"), "n_segments_aligned": g(on, "n_segments_aligned"),
                "aligned_coverage": g(on, "aligned_coverage"),
                "n_valid_frames_epoch0": g(on, "n_valid_frames_epoch0"),
                "ate_online_rmse": g(on, "ate", "rmse"), "rpe_trans_rmse": g(on, "rpe_trans", "rmse"),
                "rpe_rot_deg_rmse": g(on, "rpe_rot_deg", "rmse"),
                "lost_events": g(on, "lost_events"), "n_unrecovered": g(on, "n_unrecovered"),
                "recovery_time_median_s": g(on, "recovery_time_s", "median"),
                "ate_final_rmse": g(fi, "ate", "rmse"), "final_paired_coverage": g(fi, "paired_coverage"),
                "latency_mean_ms": lm, "latency_p95_ms": lp,
            })
    os.makedirs(os.path.join(ROOT, a.out), exist_ok=True)
    if not rows:
        print("no clean runs found under", a.roots)
        return
    with open(os.path.join(ROOT, a.out, "baseline_runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0])); w.writeheader(); w.writerows(rows)
    stats = ["coverage", "n_segments", "aligned_coverage", "ate_online_rmse", "rpe_trans_rmse", "rpe_rot_deg_rmse",
             "lost_events", "ate_final_rmse", "latency_mean_ms"]
    out = []
    keys = sorted({(r["sequence"], r["mode"], r["source"]) for r in rows},
                  key=lambda k: (k[0], MODES.index(k[1]), k[2]))
    for seq, mode, src in keys:
        grp = [r for r in rows if (r["sequence"], r["mode"], r["source"]) == (seq, mode, src)]
        row = {"sequence": seq, "mode": mode, "source": src, "n_runs": len(grp),
               "seeds": " ".join(str(r["seed"]) for r in grp),
               "n_ok": sum(r["status"] == "ok" for r in grp),
               "n_final_missing": sum(r["ate_final_rmse"] is None for r in grp),
               "alignment": grp[0]["alignment"], "segment_key": grp[0]["segment_key"]}
        for k in stats:
            v = np.array([r[k] for r in grp if r[k] is not None], float)
            row[f"{k}_n"] = len(v)
            row[f"{k}_mean"] = float(v.mean()) if len(v) else None
            row[f"{k}_std"] = float(v.std(ddof=1)) if len(v) > 1 else None
            row[f"{k}_min"] = float(v.min()) if len(v) else None
            row[f"{k}_max"] = float(v.max()) if len(v) else None
        out.append(row)
    with open(os.path.join(ROOT, a.out, "baseline_metrics.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, list(out[0])); w.writeheader(); w.writerows(out)
    for r in out:
        fmt = lambda k: "—" if r[f"{k}_mean"] is None else f"{r[f'{k}_mean']:.3f}"
        print(f"{r['sequence']:18s} {r['mode']:13s} {r['source']} n={r['n_runs']} cov={fmt('coverage')} "
              f"segs={fmt('n_segments')} ATEon={fmt('ate_online_rmse')} RPE={fmt('rpe_trans_rmse')} ATEfin={fmt('ate_final_rmse')}")


if __name__ == "__main__":
    main()
