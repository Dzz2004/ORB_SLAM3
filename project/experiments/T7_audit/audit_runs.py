"""T7 read-only audit of B/C matrix runs (never writes into results/B or results/C).

For every finished run under RUN_ROOTS: status, coverage, segments, ATE (online, segmented), n_ok_but_empty,
init_failed reason, and an in-segment jump check against GT (check_jumps.py logic: adjacent-frame relative pose
vs GT, rot > 10 deg or trans > 1.0 m; 'step' = persistent offset, 'spike' = single-frame outlier).
Then per (sequence, fault, level, mode) group over seeds: spread of coverage / ATE / segments.

Usage: audit_runs.py --out DIR [--roots results/B/runs results/C/runs] [--ate-flag 1.0] [--since-file DIR/seen.txt]
Writes DIR/runs.csv, DIR/groups.csv, DIR/flags.csv; prints a summary. Runs already audited (listed in
DIR/audited.txt) are reused from DIR/runs.csv unless their metrics.json changed.
"""
import argparse, csv, glob, json, os, re, sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
NAME = re.compile(r"^(?P<seq>office_(easy|hard)_p\d+)__(?P<fault>.+?)_s(?P<seed>\d+)__(?P<mode>MONOCULAR|IMU_MONOCULAR|RGBD|IMU_RGBD)$")


def quat2rot(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_tum(path):
    out = {}
    if not os.path.exists(path):
        return out
    for line in open(path):
        v = line.split()
        if len(v) == 8:
            T = np.eye(4); T[:3, :3] = quat2rot(np.array(v[4:8], float)); T[:3, 3] = np.array(v[1:4], float)
            out[round(float(v[0]), 3)] = T
    return out


def rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


_gt_cache = {}


def jumps(run, gt_path, mono, rows, rot_th=10.0, trans_th=1.0, min_pts=1):
    if gt_path not in _gt_cache:
        _gt_cache[gt_path] = load_tum(gt_path)
    gt, est = _gt_cache[gt_path], load_tum(os.path.join(run, "trajectory_online.tum"))
    segs = {}
    for r in rows:
        t = round(float(r["timestamp"]), 3)
        q = int(r["quality_rgb"] or 0)
        if r["valid"] == "1" and q >= min_pts and t in est and t in gt:
            segs.setdefault((r["map_id"], r.get("frame_epoch", "0")), []).append(t)
    steps, spikes, bad_segs, worst = 0, 0, set(), None
    for key, ts in segs.items():
        pairs = []
        for t0, t1 in zip(ts, ts[1:]):
            if t1 - t0 <= 0.15:
                de = np.linalg.inv(est[t0]) @ est[t1]; dg = np.linalg.inv(gt[t0]) @ gt[t1]
                pairs.append((t0, t1, de, dg))
        s = 1.0
        if mono and pairs:
            ne = np.array([np.linalg.norm(p[2][:3, 3]) for p in pairs]); ng = np.array([np.linalg.norm(p[3][:3, 3]) for p in pairs])
            m = ne > 1e-6
            s = float(np.median(ng[m] / ne[m])) if m.any() else 1.0
        for t0, t1, de, dg in pairs:
            re_ = rot_deg(de[:3, :3].T @ dg[:3, :3]); te = float(np.linalg.norm(s * de[:3, 3] - dg[:3, 3]))
            if re_ > rot_th or te > trans_th:
                i = ts.index(t1); kind = "step"
                if 1 <= i < len(ts) - 1 and ts[i + 1] - ts[i - 1] < 0.25:
                    a, b = ts[i - 1], ts[i + 1]
                    d2e = np.linalg.inv(est[a]) @ est[b]; d2g = np.linalg.inv(gt[a]) @ gt[b]
                    if rot_deg(d2e[:3, :3].T @ d2g[:3, :3]) <= rot_th and np.linalg.norm(s * d2e[:3, 3] - d2g[:3, 3]) <= trans_th:
                        kind = "spike"
                if kind == "step":
                    steps += 1; bad_segs.add(key)
                    if worst is None or te > worst[2]:
                        worst = (t1, key, te, re_)
                else:
                    spikes += 1
    return steps, spikes, len(bad_segs), worst


def init_reason(run, rows, mode):
    """Why the run never produced a valid frame (read-only from tracking.csv / run.log)."""
    log = open(os.path.join(run, "run.log"), errors="replace").read() if os.path.exists(os.path.join(run, "run.log")) else ""
    states = [r["state"] for r in rows]
    ok_empty = sum(r["state"] == "2" and int(r["quality_rgb"] or 0) == 0 for r in rows)
    qd = [float(r["quality_depth"]) for r in rows if r.get("quality_depth") not in (None, "")]
    parts = [f"states={ {s: states.count(s) for s in sorted(set(states))} }"]
    if ok_empty:
        parts.append(f"OK-but-0-points={ok_empty}")
    if qd:
        parts.append(f"depth_valid_frac_median={np.median(qd):.3f}")
    for pat in ["Not enough motion", "scale too small", "Reseting active map", "Fail to track local map",
                "TRACK_REF_KF", "not enough", "Segmentation fault", "New Map created"]:
        n = log.count(pat)
        if n:
            parts.append(f"log:'{pat}'x{n}")
    return "; ".join(parts)


def audit_run(run):
    name = os.path.basename(run)
    m = NAME.match(name)
    if not m:
        return None
    mj = os.path.join(run, "metrics.json")
    if not os.path.exists(mj):
        return None
    me = json.load(open(mj))
    fi = json.load(open(os.path.join(run, "metrics_final.json"))) if os.path.exists(os.path.join(run, "metrics_final.json")) else {}
    rows = list(csv.DictReader(open(os.path.join(run, "tracking.csv")))) if os.path.exists(os.path.join(run, "tracking.csv")) else []
    fault = m["fault"]
    lvl = me.get("level") or ""
    ftype = me.get("fault") or fault.split("_")[0]
    gt = os.path.join(ROOT, "data/seqs", m["seq"], "trajectory_gt.tum")
    st, sp, nbad, worst = (jumps(run, gt, m["mode"] == "MONOCULAR", rows,
                                 min_pts=int(me.get("min_tracked_points") or 1)) if rows else (None, None, None, None))
    status = me.get("status")
    return {
        "run": name, "root": os.path.relpath(os.path.dirname(run), ROOT), "sequence": m["seq"], "fault": ftype, "level": lvl,
        "fault_tag": fault, "seed": int(m["seed"]), "mode": m["mode"], "status": status,
        "coverage": me.get("coverage"), "n_segments": me.get("n_segments"), "n_segments_aligned": me.get("n_segments_aligned"),
        "aligned_coverage": me.get("aligned_coverage"), "n_ok_but_empty": me.get("n_ok_but_empty"),
        "n_valid_frames_epoch0": me.get("n_valid_frames_epoch0"),
        "ate_online": (me.get("ate") or {}).get("rmse"), "ate_final": (fi.get("ate") or {}).get("rmse"),
        "lost_events": me.get("lost_events"), "n_unrecovered": me.get("n_unrecovered"),
        "jump_steps": st, "jump_spikes": sp, "segments_with_step": nbad,
        "worst_step": "" if not worst else f"t={worst[0]} seg={worst[1][0]}/{worst[1][1]} trans={worst[2]:.2f}m rot={worst[3]:.1f}deg",
        "init_reason": init_reason(run, rows, m["mode"]) if status == "init_failed" or me.get("initialized") is False else "",
        "metrics_mtime": os.path.getmtime(mj),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--roots", nargs="+", default=["results/B/runs", "results/C/runs"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--ate-flag", type=float, default=1.0, help="m; online ATE above this is examined")
    a = ap.parse_args()
    out = a.out if os.path.isabs(a.out) else os.path.join(ROOT, a.out)
    os.makedirs(out, exist_ok=True)
    prev = {}
    if os.path.exists(os.path.join(out, "runs.csv")):
        for r in csv.DictReader(open(os.path.join(out, "runs.csv"))):
            prev[(r["root"], r["run"])] = r
    rows = []
    for root in a.roots:
        for run in sorted(glob.glob(os.path.join(ROOT, root, "office_*"))):
            key = (os.path.relpath(os.path.dirname(run), ROOT), os.path.basename(run))
            mj = os.path.join(run, "metrics.json")
            if key in prev and os.path.exists(mj) and abs(float(prev[key]["metrics_mtime"]) - os.path.getmtime(mj)) < 1e-3:
                rows.append(prev[key]); continue
            r = audit_run(run)
            if r:
                rows.append(r)
    if not rows:
        print("no finished runs"); return
    keys = list(rows[0])
    with open(os.path.join(out, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, keys); w.writeheader(); w.writerows(rows)

    def f(x):
        try:
            return float(x)
        except (TypeError, ValueError):
            return None

    flags = []
    for r in rows:
        why = []
        if r["status"] != "ok":
            why.append(f"status={r['status']}")
        if f(r["n_ok_but_empty"]):
            why.append(f"n_ok_but_empty={int(f(r['n_ok_but_empty']))}")
        if f(r["ate_online"]) is not None and f(r["ate_online"]) > a.ate_flag:
            why.append(f"ate_online={f(r['ate_online']):.2f}m (steps={r['jump_steps']})")
        if f(r["jump_steps"]):
            why.append(f"in-segment steps={int(f(r['jump_steps']))} in {r['segments_with_step']} segs, worst {r['worst_step']}")
        if why:
            flags.append({"root": r["root"], "run": r["run"], "issues": " | ".join(why), "init_reason": r["init_reason"]})
    with open(os.path.join(out, "flags.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, ["root", "run", "issues", "init_reason"]); w.writeheader(); w.writerows(flags)

    groups = {}
    for r in rows:
        groups.setdefault((r["root"], r["sequence"], r["fault_tag"], r["mode"]), []).append(r)
    grows = []
    for (root, seq, fault, mode), g in sorted(groups.items()):
        row = {"root": root, "sequence": seq, "fault_tag": fault, "mode": mode, "n_seeds": len(g),
               "seeds": " ".join(sorted(str(x["seed"]) for x in g)), "n_not_ok": sum(x["status"] != "ok" for x in g)}
        for k in ("coverage", "ate_online", "n_segments", "ate_final"):
            v = np.array([f(x[k]) for x in g if f(x[k]) is not None])
            row[f"{k}_min"] = float(v.min()) if len(v) else None
            row[f"{k}_max"] = float(v.max()) if len(v) else None
            row[f"{k}_ratio"] = float(v.max() / v.min()) if len(v) > 1 and v.min() > 0 else None
        grows.append(row)
    with open(os.path.join(out, "groups.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, list(grows[0])); w.writeheader(); w.writerows(grows)

    from collections import Counter
    print(f"runs audited: {len(rows)}  status: {dict(Counter(r['status'] for r in rows))}")
    print(f"runs with in-segment steps: {sum(1 for r in rows if f(r['jump_steps']))}  "
          f"n_ok_but_empty>0: {sum(1 for r in rows if f(r['n_ok_but_empty']))}  "
          f"ate_online>{a.ate_flag}: {sum(1 for r in rows if (f(r['ate_online']) or 0) > a.ate_flag)}")
    print("flags:")
    for fl in flags:
        print(f"  {fl['root']}/{fl['run']}: {fl['issues']}" + (f"  [init: {fl['init_reason']}]" if fl["init_reason"] else ""))
    print("seed spread (groups with >=2 seeds, ate ratio>3 or coverage range>0.2):")
    for g in grows:
        if g["n_seeds"] >= 2 and ((g["ate_online_ratio"] or 0) > 3 or ((g["coverage_max"] or 0) - (g["coverage_min"] or 0)) > 0.2):
            print(f"  {g['root']} {g['sequence']} {g['fault_tag']} {g['mode']} seeds={g['seeds']}: "
                  f"cov {g['coverage_min']:.3f}-{g['coverage_max']:.3f}, ATE {g['ate_online_min']}-{g['ate_online_max']}, "
                  f"segs {g['n_segments_min']}-{g['n_segments_max']}")


if __name__ == "__main__":
    main()
