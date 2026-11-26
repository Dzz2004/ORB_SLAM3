"""At each frame_epoch increment (event frame t, from frame_events.csv), is the pose output at t still in the
OLD world frame? Compare the estimated step t-1 -> t and t -> t+1 with GT (translation error after per-run scale
for MONOCULAR is skipped: only SE3 modes are tested). Prints per-run counts:
  old_at_event : error(t-1->t) small, error(t->t+1) large  -> frame t belongs to the previous epoch
  new_at_event : error(t-1->t) large, error(t->t+1) small  -> frame t is already in the new frame (current rule OK)
  no_jump      : both small (frame change too small to see at 0.3 m)
Usage: event_frame_check.py RUN_DIR [RUN_DIR ...]"""
import csv, json, os, sys
import numpy as np

def tum(p):
    return {round(float(l.split()[0]), 3): np.array(l.split()[1:4], float) for l in open(p) if l.strip()}

tot = {"old_at_event": 0, "new_at_event": 0, "no_jump": 0, "both": 0, "untestable": 0}
for run in sys.argv[1:]:
    info = os.path.join(run, "run_info.json")
    if not os.path.exists(info) or not os.path.exists(os.path.join(run, "frame_events.csv")):
        continue  # run still in progress
    mode = json.load(open(info))["mode"]
    if mode == "MONOCULAR" or not os.path.exists(os.path.join(run, "frame_events.csv")):
        continue
    seq = json.load(open(info))["sequence"]
    base = os.path.join(os.path.dirname(seq), os.path.basename(seq).split("__")[0])  # corrupted seqs have no GT copy
    gt = tum(os.path.join(base, "trajectory_gt.tum"))
    est = tum(os.path.join(run, "trajectory_online.tum"))
    rows = list(csv.DictReader(open(os.path.join(run, "tracking.csv"))))
    ok = [round(float(r["timestamp"]), 3) for r in rows if r["valid"] == "1"]
    pos = {t: i for i, t in enumerate(ok)}
    c = dict.fromkeys(tot, 0)
    for e in csv.DictReader(open(os.path.join(run, "frame_events.csv"))):
        if e["reason"] == "reinit":
            continue
        t = round(float(e["timestamp"]), 3); i = pos.get(t)
        if i is None or i == 0 or i + 1 >= len(ok) or ok[i] - ok[i - 1] > 0.15 or ok[i + 1] - ok[i] > 0.15 \
                or any(x not in est or x not in gt for x in (ok[i - 1], ok[i], ok[i + 1])):
            c["untestable"] += 1; continue
        a, b, n = ok[i - 1], ok[i], ok[i + 1]
        e1 = np.linalg.norm((est[b] - est[a]) - (gt[b] - gt[a]))  # crude: valid only if world frames ~aligned locally
        e2 = np.linalg.norm((est[n] - est[b]) - (gt[n] - gt[b]))
        d1 = abs(np.linalg.norm(est[b] - est[a]) - np.linalg.norm(gt[b] - gt[a]))  # rotation-invariant step length error
        d2 = abs(np.linalg.norm(est[n] - est[b]) - np.linalg.norm(gt[n] - gt[b]))
        big1, big2 = d1 > 0.3, d2 > 0.3
        k = "both" if big1 and big2 else "old_at_event" if big2 else "new_at_event" if big1 else "no_jump"
        c[k] += 1
    print(os.path.basename(run.rstrip("/")), c)
    for k in tot:
        tot[k] += c[k]
print("TOTAL", tot)
