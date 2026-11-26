"""Summarise T1b concurrency runs: per (level, mode) coverage, map segments, online ATE (evaluate.py
--segment-by-map) and latency, mean/std/min/max over repeats. Usage: summarize_concurrency.py RESULT_DIR
Writes RESULT_DIR/summary.csv and RESULT_DIR/runs.csv; runs evaluate.py for each run lacking metrics_online.json."""
import csv, json, os, subprocess, sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
GT = os.path.join(ROOT, "data/seqs/office_easy_p004/trajectory_gt.tum")


def evaluate(run, mode):
    out = os.path.join(run, "metrics_online.json")
    if not os.path.exists(out):
        subprocess.run([sys.executable, "-I", os.path.join(ROOT, "scripts/evaluate.py"),
                        "--est", os.path.join(run, "trajectory_online.tum"), "--gt", GT,
                        "--tracking", os.path.join(run, "tracking.csv"), "--mode", mode, "--out", out,
                        "--segment-by-map", "--seed", "0"], check=True, capture_output=True)
    return json.load(open(out))


def find(d, *keys):
    """First matching key anywhere in a nested dict."""
    if isinstance(d, dict):
        for k in keys:
            if k in d and not isinstance(d[k], dict):
                return d[k]
        for v in d.values():
            r = find(v, *keys)
            if r is not None:
                return r
    return None


def main():
    base = sys.argv[1]
    runs = []
    for lvl in sorted(os.listdir(base)):
        if not lvl.startswith("c") or not os.path.isdir(os.path.join(base, lvl)):
            continue
        for name in sorted(os.listdir(os.path.join(base, lvl))):
            run = os.path.join(base, lvl, name)
            mode = name.split("__")[0]
            rows = list(csv.DictReader(open(os.path.join(run, "tracking.csv"))))
            info = json.load(open(os.path.join(run, "run_info.json")))
            lat = np.array([float(r["latency_ms"]) for r in rows])
            ok = sum(r["valid"] == "1" for r in rows)
            m = evaluate(run, mode)
            runs.append({"level": int(lvl[1:]), "mode": mode, "run": name, "exit": info["exit_code"],
                         "frames": len(rows), "coverage": ok / len(rows),
                         "map_segments": len({r["map_id"] for r in rows if r["valid"] == "1"}),
                         "ate_online_rmse": (m.get("ate") or {}).get("rmse"),
                         "lat_mean_ms": lat.mean(), "lat_p95_ms": np.percentile(lat, 95)})
    keys = list(runs[0])
    with open(os.path.join(base, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, keys); w.writeheader(); w.writerows(runs)
    out = []
    for lvl in sorted({r["level"] for r in runs}):
        for mode in ("IMU_RGBD", "IMU_MONOCULAR"):
            g = [r for r in runs if r["level"] == lvl and r["mode"] == mode]
            if not g:
                continue
            row = {"level": lvl, "mode": mode, "n_runs": len(g), "n_failed_exit": sum(r["exit"] != 0 for r in g)}
            for k in ("coverage", "map_segments", "ate_online_rmse", "lat_mean_ms", "lat_p95_ms"):
                v = np.array([r[k] for r in g if r[k] is not None], float)
                row.update({f"{k}_mean": v.mean(), f"{k}_std": v.std(ddof=1) if len(v) > 1 else 0.0,
                            f"{k}_min": v.min(), f"{k}_max": v.max()})
            out.append(row)
    with open(os.path.join(base, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, list(out[0])); w.writeheader(); w.writerows(out)
    for r in out:
        print("c%-3d %-13s n=%d cov %.3f±%.3f [%.3f,%.3f]  segs %.1f±%.1f [%d,%d]  ATE %.3f±%.3f [%.3f,%.3f]  lat %.0f/%.0f ms"
              % (r["level"], r["mode"], r["n_runs"], r["coverage_mean"], r["coverage_std"], r["coverage_min"],
                 r["coverage_max"], r["map_segments_mean"], r["map_segments_std"], r["map_segments_min"],
                 r["map_segments_max"], r["ate_online_rmse_mean"], r["ate_online_rmse_std"],
                 r["ate_online_rmse_min"], r["ate_online_rmse_max"], r["lat_mean_ms_mean"], r["lat_p95_ms_mean"]))


if __name__ == "__main__":
    main()
