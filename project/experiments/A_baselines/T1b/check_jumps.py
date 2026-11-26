"""Check that each (map_id, frame_epoch) segment of trajectory_online.tum has no world-frame jump.

For consecutive OK frames i-1, i in the same segment, compare the estimated relative motion
dT_est = T_{i-1}^-1 T_i with the ground-truth one. A world-frame jump shows up as a large rotation
error of the relative motion. Monocular translation is scale-free, so translation is compared after
per-segment median scale. Thresholds are set well above per-frame tracking noise
(p99 rotation error is 0.3-9 deg, translation 0.3-0.7 m is common on hard), so only frame changes are flagged.
A flagged frame is a "spike" (single-frame tracking outlier, the motion t-1 -> t+1 agrees with GT) or a
"step" (persistent offset = an undetected world-frame change). Acceptance: no steps.
Usage: check_jumps.py RUN_DIR GT_TUM [--rot-th 10] [--trans-th 1.0]
Prints one JSON line per run; with --list also prints the offending frames.
"""
import argparse, csv, json, os, sys
import numpy as np


def quat2rot(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_tum(path):
    out = {}
    for line in open(path):
        v = line.split()
        if len(v) == 8:
            T = np.eye(4)
            T[:3, :3] = quat2rot(np.array(v[4:8], float))
            T[:3, 3] = np.array(v[1:4], float)
            out[round(float(v[0]), 3)] = T
    return out


def rot_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("gt")
    ap.add_argument("--rot-th", type=float, default=10.0, help="deg, relative-rotation error flagged as jump")
    ap.add_argument("--trans-th", type=float, default=1.0, help="m, relative-translation error flagged as jump")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    est, gt = load_tum(os.path.join(a.run, "trajectory_online.tum")), load_tum(a.gt)
    rows = list(csv.DictReader(open(os.path.join(a.run, "tracking.csv"))))
    has_epoch = "frame_epoch" in rows[0]
    mono = json.load(open(os.path.join(a.run, "run_info.json")))["mode"] == "MONOCULAR"
    segs = {}
    for r in rows:
        t = round(float(r["timestamp"]), 3)
        if r["valid"] == "1" and t in est and t in gt:
            key = (r["map_id"], r["frame_epoch"] if has_epoch else "0")
            segs.setdefault(key, []).append(t)
    pairs = []  # (t, seg, rot_err, trans_err_raw, d_est, d_gt)
    for key, ts in segs.items():
        for t0, t1 in zip(ts, ts[1:]):
            if t1 - t0 > 0.15:  # only adjacent frames (10 Hz)
                continue
            de = np.linalg.inv(est[t0]) @ est[t1]
            dg = np.linalg.inv(gt[t0]) @ gt[t1]
            pairs.append((t1, key, rot_deg(de[:3, :3].T @ dg[:3, :3]), de[:3, 3], dg[:3, 3]))
    bad = []
    by_seg = {}
    for p in pairs:
        by_seg.setdefault(p[1], []).append(p)
    for key, ps in by_seg.items():
        s = 1.0
        if mono:
            ne = np.array([np.linalg.norm(p[3]) for p in ps]); ng = np.array([np.linalg.norm(p[4]) for p in ps])
            m = ne > 1e-6
            s = float(np.median(ng[m] / ne[m])) if m.any() else 1.0
        flagged = []
        for t, k, re, te, tg in ps:
            terr = float(np.linalg.norm(s * te - tg))
            if re > a.rot_th or terr > a.trans_th:
                flagged.append({"t": t, "map_id": k[0], "frame_epoch": k[1], "rot_err_deg": round(re, 2),
                                "trans_err_m": round(terr, 3)})
        # A single-frame tracking outlier (t) is followed by an opposite error at t+1: the motion t-1 -> t+1
        # agrees with GT. A world-frame change is a step: it does not cancel.
        ts = segs[key]
        for f in flagged:
            i = ts.index(f["t"])
            f["kind"] = "step"
            if 1 <= i < len(ts) - 1 and ts[i + 1] - ts[i - 1] < 0.25:
                t0, t2 = ts[i - 1], ts[i + 1]
                de = np.linalg.inv(est[t0]) @ est[t2]; dg = np.linalg.inv(gt[t0]) @ gt[t2]
                if rot_deg(de[:3, :3].T @ dg[:3, :3]) <= a.rot_th and np.linalg.norm(s * de[:3, 3] - dg[:3, 3]) <= a.trans_th:
                    f["kind"] = "spike"
        bad += flagged
    rots = np.array([p[2] for p in pairs]) if pairs else np.zeros(1)
    print(json.dumps({"run": os.path.basename(os.path.normpath(a.run)), "has_frame_epoch": has_epoch,
                      "segments": len(segs), "adjacent_pairs": len(pairs), "jumps": len(bad), "steps": sum(b["kind"] == "step" for b in bad),
                      "spikes": sum(b["kind"] == "spike" for b in bad),
                      "segments_with_step": len({(b["map_id"], b["frame_epoch"]) for b in bad if b["kind"] == "step"}),
                      "rot_err_deg_p50": round(float(np.median(rots)), 3),
                      "rot_err_deg_p99": round(float(np.percentile(rots, 99)), 3),
                      "rot_err_deg_max": round(float(rots.max()), 2)}))
    if a.list:
        for b in bad:
            print("  JUMP", json.dumps(b))


if __name__ == "__main__":
    sys.exit(main())
