"""For each 'step' flagged by check_jumps.py, test whether it is a world-frame change.
Within its (map_id, frame_epoch) segment, align the whole segment once (joint) and the parts before/after the
flagged frame separately (split). A world-frame change makes joint RMSE >> split RMSE; a local tracking error
does not. Reports ratio joint/split and the RMSE gain. Usage: split_test.py RUN_DIR GT_TUM"""
import csv, json, os, subprocess, sys
import numpy as np


def tum(p):
    return {round(float(l.split()[0]), 3): np.array(l.split()[1:4], float) for l in open(p) if l.strip()}


def sq_err(X, Y, scale):
    mx, my = X.mean(1, keepdims=True), Y.mean(1, keepdims=True)
    U, D, V = np.linalg.svd((Y - my) @ (X - mx).T / X.shape[1]); S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(V) < 0: S[2, 2] = -1
    s = np.trace(np.diag(D) @ S) / ((X - mx) ** 2).sum(0).mean() if scale else 1.0
    return ((s * U @ S @ V @ (X - mx) - (Y - my)) ** 2).sum(0)


run, gtp = sys.argv[1], sys.argv[2]
mono = json.load(open(os.path.join(run, "run_info.json")))["mode"] == "MONOCULAR"
est, gt = tum(os.path.join(run, "trajectory_online.tum")), tum(gtp)
rows = list(csv.DictReader(open(os.path.join(run, "tracking.csv"))))
segs = {}
for r in rows:
    t = round(float(r["timestamp"]), 3)
    if r["valid"] == "1" and t in est and t in gt:
        segs.setdefault((r["map_id"], r["frame_epoch"]), []).append(t)
out = subprocess.run([sys.executable, "-I", os.path.join(os.path.dirname(__file__), "check_jumps.py"), run, gtp, "--list"],
                     capture_output=True, text=True).stdout
steps = [json.loads(l.split("JUMP ", 1)[1]) for l in out.splitlines() if "JUMP" in l]
steps = [s for s in steps if s["kind"] == "step"]
seen = set()
for s in steps:
    key = (s["map_id"], s["frame_epoch"]); ts = segs[key]; i = ts.index(s["t"])
    if (key, i) in seen: continue
    seen.add((key, i))
    A, B = ts[:i], ts[i:]
    X = lambda k: np.array([est[t] for t in k]).T
    Y = lambda k: np.array([gt[t] for t in k]).T
    joint = np.sqrt(sq_err(X(ts), Y(ts), mono).mean())
    if len(A) >= 4 and len(B) >= 4:
        split = np.sqrt(np.concatenate([sq_err(X(A), Y(A), mono), sq_err(X(B), Y(B), mono)]).mean())
        verdict = "FRAME_CHANGE" if joint > 3 * split and joint - split > 0.3 else "tracking_error"
    else:
        split, verdict = float("nan"), "too_short_to_test"
    print(json.dumps({"run": os.path.basename(run), "t": s["t"], "seg": key, "n_before": len(A), "n_after": len(B),
                      "rmse_joint": round(float(joint), 3), "rmse_split": round(float(split), 3), "verdict": verdict}))
