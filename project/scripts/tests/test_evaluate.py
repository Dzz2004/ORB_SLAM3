#!/usr/bin/env python
"""Self-test for scripts/evaluate.py using a GT trajectory with known perturbations.

Usage: python scripts/tests/test_evaluate.py data/seqs/office_easy_p004 OUT_DIR
"""
import json
import os
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
EVAL = os.path.join(HERE, "..", "evaluate.py")


def write_tum(path, t, P, Q):
    with open(path, "w") as f:
        for a, p, q in zip(t, P, Q):
            f.write(f"{a:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


def write_trk(path, t, valid, src=None):
    with open(path, "w") as f:
        f.write("timestamp,valid,state,quality_rgb,quality_depth,source,latency_ms\n")
        for k, (a, v) in enumerate(zip(t, valid)):
            s = src[k] if src else ("SLAM" if v else "NONE")
            f.write(f"{a:.6f},{int(v)},{2 if v else 4},100,,{s},10.0\n")


def run(out, case, est, trk, mode, extra=()):
    mj = os.path.join(out, case + ".json")
    subprocess.run([sys.executable, EVAL, "--est", est, "--gt", GT, "--tracking", trk, "--mode", mode,
                    "--out", mj, "--seed", "0", *extra], check=True, stdout=subprocess.DEVNULL)
    return json.load(open(mj))


seq, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
GT = os.path.join(seq, "trajectory_gt.tum")
g = np.loadtxt(GT)
t, P, Q = g[:, 0], g[:, 1:4], g[:, 4:]
n = len(t)
rng = np.random.default_rng(0)
trk_all = os.path.join(out, "trk_all.csv")
write_trk(trk_all, t, [True] * n)
fails = []


def check(name, cond, val):
    print(("PASS" if cond else "FAIL"), name, val)
    if not cond:
        fails.append(name)


# 1) identity
write_tum(os.path.join(out, "id.tum"), t, P, Q)
m = run(out, "id", os.path.join(out, "id.tum"), trk_all, "IMU_RGBD")
check("identity ATE~0", m["ate"]["rmse"] < 1e-5, m["ate"]["rmse"])
check("identity RPE~0", m["rpe_trans"]["rmse"] < 1e-5 and m["rpe_rot_deg"]["rmse"] < 1e-3,
      (m["rpe_trans"]["rmse"], m["rpe_rot_deg"]["rmse"]))
check("identity coverage=1, 0 lost", m["coverage"] == 1 and m["lost_events"] == 0, (m["coverage"], m["lost_events"]))

# 2) rigid transform of whole trajectory: SE3 alignment must remove it
Rg = R.from_euler("xyz", [30, -20, 70], degrees=True)
P2 = Rg.apply(P) + np.array([5.0, -3.0, 2.0])
Q2 = (Rg * R.from_quat(Q)).as_quat()
write_tum(os.path.join(out, "rigid.tum"), t, P2, Q2)
m = run(out, "rigid", os.path.join(out, "rigid.tum"), trk_all, "IMU_RGBD")
check("rigid SE3 ATE~0", m["ate"]["rmse"] < 1e-4, m["ate"]["rmse"])

# 3) similarity (scale 0.37): MONOCULAR auto -> sim3 recovers it; SE3 does not
P3 = 0.37 * P2
write_tum(os.path.join(out, "sim.tum"), t, P3, Q2)
m = run(out, "sim_mono", os.path.join(out, "sim.tum"), trk_all, "MONOCULAR")
check("sim3 ATE~0 & scale=1/0.37",
      m["alignment"] == "sim3" and m["ate"]["rmse"] < 1e-4 and abs(m["scale"] - 1 / 0.37) < 1e-3,
      (m["ate"]["rmse"], m["scale"]))
check("sim3 RPE~0 after scale", m["rpe_trans"]["rmse"] < 1e-4, m["rpe_trans"]["rmse"])
m = run(out, "sim_se3", os.path.join(out, "sim.tum"), trk_all, "IMU_MONOCULAR")
check("scaled traj under SE3 has large ATE", m["alignment"] == "se3" and m["ate"]["rmse"] > 1.0, m["ate"]["rmse"])

# 4) gaussian position noise sigma per axis -> ATE rmse ~ sigma*sqrt(3), RPE trans ~ sigma*sqrt(6)
sig = 0.05
P4 = P2 + rng.normal(0, sig, P.shape)
write_tum(os.path.join(out, "noise.tum"), t, P4, Q2)
m = run(out, "noise", os.path.join(out, "noise.tum"), trk_all, "RGBD")
exp = sig * np.sqrt(3)
check(f"noise ATE ~ {exp:.4f}", abs(m["ate"]["rmse"] - exp) / exp < 0.05, m["ate"]["rmse"])
exp_rpe = sig * np.sqrt(6)
check(f"noise RPE ~ {exp_rpe:.4f}", abs(m["rpe_trans"]["rmse"] - exp_rpe) / exp_rpe < 0.08, m["rpe_trans"]["rmse"])

# 5) rotation noise 1 deg per axis on each pose -> RPE rot rmse ~ sqrt(6) deg
Q5 = (R.from_rotvec(rng.normal(0, np.radians(1.0), (n, 3))) * R.from_quat(Q2)).as_quat()
write_tum(os.path.join(out, "rotnoise.tum"), t, P2, Q5)
m = run(out, "rotnoise", os.path.join(out, "rotnoise.tum"), trk_all, "RGBD")
exp_r = np.sqrt(6) * 1.0
check(f"rot noise RPE rot ~ {exp_r:.2f} deg", abs(m["rpe_rot_deg"]["rmse"] - exp_r) / exp_r < 0.1, m["rpe_rot_deg"]["rmse"])

# 6) lost segments: init delay 10 frames; lost [100,120) recovers; lost [500,502), 1 valid frame,
#    lost [503,510) -> recovery needs 3 consecutive valid; lost from n-30 never recovers
v = np.ones(n, bool)
v[:10] = False
v[100:120] = False
v[500:502] = False
v[503:510] = False
v[n - 30:] = False
src = ["SLAM" if x else "NONE" for x in v]
for k in range(100, 105):
    src[k] = "IMU"
trk = os.path.join(out, "trk_lost.csv")
write_trk(trk, t, v, src)
keep = np.nonzero(v)[0]
write_tum(os.path.join(out, "lost.tum"), t[keep], P2[keep], Q2[keep])
m = run(out, "lost", os.path.join(out, "lost.tum"), trk, "IMU_RGBD")
ev = m["events"]
check("init delay 1.0s", abs(m["init_delay_s"] - 1.0) < 1e-6, m["init_delay_s"])
check("4 lost events", m["lost_events"] == 4, m["lost_events"])
check("event1 recovery 2.0s", ev[0]["recovered"] and abs(ev[0]["recovery_time_s"] - 2.0) < 1e-6, ev[0])
check("event1 imu frames 5", ev[0]["imu_predicted_frames"] == 5, ev[0]["imu_predicted_frames"])
check("event2 recovers only at frame 510 -> 1.0s",
      ev[1]["recovered"] and abs(ev[1]["recovery_time_s"] - 1.0) < 1e-6, ev[1])
check("last event unrecovered, null not 0", (not ev[-1]["recovered"]) and ev[-1]["recovery_time_s"] is None, ev[-1])
check("n_unrecovered 1", m["n_unrecovered"] == 1, m["n_unrecovered"])
check("coverage", abs(m["coverage"] - v.mean()) < 1e-9, (m["coverage"], v.mean()))
check("ATE on valid paired only ~0", m["ate"]["rmse"] < 1e-4 and m["n_paired"] == len(keep),
      (m["ate"]["rmse"], m["n_paired"]))

# 7) fault interval recovery: fault [10.0, 12.0) == frames 100..119, valid again at 12.0
fi = os.path.join(out, "faults.csv")
with open(fi, "w") as f:
    f.write("start,end\n10.0,12.0\n")
m = run(out, "fault", os.path.join(out, "lost.tum"), trk, "IMU_RGBD", ["--fault-intervals", fi])
fr = m["fault_recovery"][0]
check("fault recovery 0s", fr["recovered"] and abs(fr["recovery_time_s"]) < 1e-6 and fr["valid_frames_during_fault"] == 0, fr)

# 8) error-gated recovery: 2 m error on frames 120..129 -> recovery at frame 130 (3.0 s after loss)
P8 = P2.copy()
P8[120:130] += np.array([2.0, 0, 0])
write_tum(os.path.join(out, "lost_err.tum"), t[keep], P8[keep], Q2[keep])
m = run(out, "lost_err", os.path.join(out, "lost_err.tum"), trk, "IMU_RGBD", ["--recover-err", "0.5"])
check("error-gated recovery 3.0s", abs(m["events"][0]["recovery_time_s"] - 3.0) < 1e-6, m["events"][0]["recovery_time_s"])

# 9) empty estimate (initialisation failure)
open(os.path.join(out, "empty.tum"), "w").close()
trk0 = os.path.join(out, "trk_none.csv")
write_trk(trk0, t, [False] * n)
m = run(out, "empty", os.path.join(out, "empty.tum"), trk0, "IMU_RGBD")
check("empty: not initialized, ATE None, 0 events",
      (not m["initialized"]) and m["ate"] is None and m["lost_events"] == 0 and m["coverage"] == 0,
      (m["initialized"], m["ate"], m["lost_events"]))

# 10) map segments: three maps with different rigid frames (map 2 also scaled), short map 3
def write_trk_map(path, t, valid, mids):
    with open(path, "w") as f:
        f.write("timestamp,valid,state,quality_rgb,quality_depth,source,latency_ms,map_id\n")
        for a_, v_, mid in zip(t, valid, mids):
            f.write(f"{a_:.6f},{int(v_)},{2 if v_ else 4},100,,{'SLAM' if v_ else 'NONE'},10.0,{mid}\n")


v = np.ones(n, bool)
v[400:420] = False
v[800:810] = False
mids = np.full(n, 0)
mids[400:] = 1
mids[800:] = 2
mids[810 + 2:] = 3  # map 2 has only 2 valid poses (810, 811) -> not aligned
mids[~v] = -1
Pm, Qm = P.copy(), Q.copy()
for mid, (ang, off, sc) in {0: ([0, 0, 0], [0, 0, 0], 1.0), 1: ([10, 50, -30], [3, 1, -2], 1.0),
                            2: ([0, 0, 90], [9, 9, 9], 1.0), 3: ([-40, 5, 120], [-4, 2, 7], 1.0)}.items():
    k = mids == mid
    Rk = R.from_euler("xyz", ang, degrees=True)
    Pm[k] = sc * Rk.apply(P[k]) + np.array(off)
    Qm[k] = (Rk * R.from_quat(Q[k])).as_quat()
keep = np.nonzero(v)[0]
sig = 0.02
Pm_noisy = Pm.copy()
Pm_noisy[keep] += rng.normal(0, sig, (len(keep), 3))
write_tum(os.path.join(out, "maps.tum"), t[keep], Pm_noisy[keep], Qm[keep])
trkm = os.path.join(out, "trk_maps.csv")
write_trk_map(trkm, t, v, mids)
m = run(out, "maps_global", os.path.join(out, "maps.tum"), trkm, "IMU_RGBD")
check("maps without segmentation: large ATE", m["ate"]["rmse"] > 1.0, m["ate"]["rmse"])
m = run(out, "maps_seg", os.path.join(out, "maps.tum"), trkm, "IMU_RGBD", ["--segment-by-map", "--drop-segment-first-frames", "0"])
segs = m["segments"]
check("4 segments, 3 aligned", m["n_segments"] == 4 and m["n_segments_aligned"] == 3,
      (m["n_segments"], m["n_segments_aligned"]))
check("segment map_ids in order", [s["map_id"] for s in segs] == ["0", "1", "2", "3"], [s["map_id"] for s in segs])
check("short segment not aligned", (not segs[2]["aligned"]) and segs[2]["n"] == 2 and segs[2]["ate"] is None, segs[2])
exp = sig * np.sqrt(3)
check(f"segmented ATE ~ {exp:.4f}", abs(m["ate"]["rmse"] - exp) / exp < 0.08, m["ate"]["rmse"])
n_al = sum(s["n"] for s in segs if s["aligned"])
pooled = np.sqrt(sum(s["n"] * s["ate"]["rmse"] ** 2 for s in segs if s["aligned"]) / n_al)
check("overall ATE = frame-weighted pool of segments", abs(pooled - m["ate"]["rmse"]) < 1e-9 and m["ate"]["n"] == n_al,
      (pooled, m["ate"]["rmse"], m["ate"]["n"], n_al))
check("aligned_coverage", abs(m["aligned_coverage"] - n_al / n) < 1e-9, m["aligned_coverage"])
check("RPE does not cross segments (small)", m["rpe_trans"]["rmse"] < 0.1, m["rpe_trans"]["rmse"])

# 11) map segments with sim3: map 1 at scale 0.5 in MONOCULAR
Pm2 = Pm.copy()
k1 = mids == 1
Pm2[k1] = 0.5 * Pm[k1]
write_tum(os.path.join(out, "maps_sim.tum"), t[keep], Pm2[keep], Qm[keep])
m = run(out, "maps_sim", os.path.join(out, "maps_sim.tum"), trkm, "MONOCULAR", ["--segment-by-map", "--drop-segment-first-frames", "0"])
check("sim3 per-segment scale", abs(m["segments"][1]["scale"] - 2.0) < 1e-3 and m["ate"]["rmse"] < 1e-4,
      (m["segments"][1]["scale"], m["ate"]["rmse"]))

# 12) --segment-by-map without map_id column must fail
r = subprocess.run([sys.executable, EVAL, "--est", os.path.join(out, "id.tum"), "--gt", GT, "--tracking", trk_all,
                    "--mode", "RGBD", "--out", os.path.join(out, "x.json"), "--segment-by-map"],
                   capture_output=True)
check("missing map_id column -> error", r.returncode != 0, r.returncode)

# 13) frame_epoch: one map, world frame rotated 105 deg at frame 30 (IMU init) and shifted at 300 (VIBA)
def write_trk_epoch(path, t, mids, eps):
    with open(path, "w") as f:
        f.write("timestamp,valid,state,quality_rgb,quality_depth,source,latency_ms,map_id,frame_epoch\n")
        for a_, mid, e in zip(t, mids, eps):
            v_ = mid >= 0
            f.write(f"{a_:.6f},{int(v_)},{2 if v_ else 1},100,,{'SLAM' if v_ else 'NONE'},10.0,{mid},{e}\n")


mids = np.zeros(n, int)
mids[:3] = -1
eps = np.zeros(n, int)
eps[:3] = -1
eps[30:] = 1
eps[300:] = 2
Pe, Qe = P.copy(), Q.copy()
for e_, (ang, off) in {1: ([0, 0, 105], [1, 2, 3]), 2: ([5, -3, 100], [1.5, 2, 3])}.items():
    k = eps == e_
    Rk = R.from_euler("xyz", ang, degrees=True)
    Pe[k] = Rk.apply(P[k]) + np.array(off)
    Qe[k] = (Rk * R.from_quat(Q[k])).as_quat()
keep = np.nonzero(mids >= 0)[0]
write_tum(os.path.join(out, "epoch.tum"), t[keep], Pe[keep], Qe[keep])
trke = os.path.join(out, "trk_epoch.csv")
write_trk_epoch(trke, t, mids, eps)
# map_id-only behaviour (drop the column) must show the jump
with open(trke) as f, open(os.path.join(out, "trk_epoch_nocol.csv"), "w") as g:
    for line in f:
        g.write(",".join(line.rstrip("\n").split(",")[:8]) + "\n")
m = run(out, "epoch_nocol", os.path.join(out, "epoch.tum"), os.path.join(out, "trk_epoch_nocol.csv"), "IMU_RGBD",
        ["--segment-by-map", "--drop-segment-first-frames", "0"])
check("map_id only: world-frame jump gives large ATE", m["ate"]["rmse"] > 1.0 and m["n_segments"] == 1,
      (m["ate"]["rmse"], m["n_segments"]))
m = run(out, "epoch", os.path.join(out, "epoch.tum"), trke, "IMU_RGBD", ["--segment-by-map", "--drop-segment-first-frames", "0"])
check("(map_id, frame_epoch): 3 segments, ATE~0",
      m["segment_key"] == "map_id,frame_epoch" and m["n_segments"] == 3 and m["ate"]["rmse"] < 1e-4,
      (m.get("segment_key"), m["n_segments"], m["ate"]["rmse"]))
check("segment_frames [27, 270, n-300]", m["segment_frames"] == [27, 270, n - 300], m["segment_frames"])
check("n_valid_frames_epoch0 = 27", m["n_valid_frames_epoch0"] == 27, m["n_valid_frames_epoch0"])

# 14) degenerate segment (all est poses identical, as after an in-frame map reset) must not crash
mids = np.zeros(n, int)
mids[n - 4:] = 1
eps = np.zeros(n, int)
Pd, Qd = P.copy(), Q.copy()
Pd[n - 4:] = 0.0
Qd[n - 4:] = [0, 0, 0, 1]
write_tum(os.path.join(out, "degen.tum"), t, Pd, Qd)
trkd = os.path.join(out, "trk_degen.csv")
write_trk_epoch(trkd, t, mids, eps)
m = run(out, "degen", os.path.join(out, "degen.tum"), trkd, "RGBD", ["--segment-by-map", "--drop-segment-first-frames", "0"])
check("degenerate segment skipped, counted, rest evaluated",
      m["n_segments"] == 2 and m["n_segments_degenerate"] == 1 and m["segments"][1]["reason"] == "degenerate"
      and m["ate"]["rmse"] < 1e-4 and m["ate"]["n"] == n - 4,
      (m["n_segments"], m.get("n_segments_degenerate"), m["ate"]["rmse"], m["ate"]["n"]))

# 15) --min-tracked-points: OK frames with 0 tracked points (empty-map "initialisation", as in depth_off)
#     frames 200..259 alternate OK/0-points and LOST; the 0-point OK frames have the pose at the origin
trk_e = os.path.join(out, "trk_empty.csv")
pts = np.full(n, 100)
st = np.full(n, 2)
for k in range(200, 260):
    if k % 2 == 0:
        pts[k] = 0
    else:
        st[k] = 4
        pts[k] = 0
with open(trk_e, "w") as f:
    f.write("timestamp,valid,state,quality_rgb,quality_depth,source,latency_ms,map_id,frame_epoch\n")
    for k in range(n):
        v_ = st[k] == 2
        f.write(f"{t[k]:.6f},{int(v_)},{st[k]},{pts[k]},,{'SLAM' if v_ else 'NONE'},10.0,0,0\n")
Pz, Qz = P2.copy(), Q2.copy()
okk = np.nonzero(st == 2)[0]
for k in range(200, 260, 2):
    Pz[k] = 0.0
    Qz[k] = [0, 0, 0, 1]
write_tum(os.path.join(out, "empty_map.tum"), t[okk], Pz[okk], Qz[okk])
m = run(out, "empty_map_old", os.path.join(out, "empty_map.tum"), trk_e, "RGBD",
        ["--segment-by-map", "--min-tracked-points", "0", "--drop-segment-first-frames", "0"])
check("threshold 0: empty-map frames counted valid, ATE polluted",
      m["n_ok_but_empty"] == 0 and m["lost_events"] == 30 and m["ate"]["rmse"] > 1.0,
      (m["n_ok_but_empty"], m["lost_events"], m["ate"]["rmse"]))
m = run(out, "empty_map", os.path.join(out, "empty_map.tum"), trk_e, "RGBD", ["--segment-by-map", "--drop-segment-first-frames", "0"])
check("default threshold 1: n_ok_but_empty=30, one lost event 6.0 s, ATE clean",
      m["min_tracked_points"] == 1 and m["n_ok_but_empty"] == 30 and m["lost_events"] == 1
      and abs(m["events"][0]["lost_duration_s"] - 6.0) < 1e-6 and m["ate"]["rmse"] < 1e-4
      and m["n_est_dropped_not_valid"] == 30 and abs(m["coverage"] - (n - 60) / n) < 1e-9,
      (m["min_tracked_points"], m["n_ok_but_empty"], m["lost_events"], m["ate"]["rmse"],
       m.get("n_est_dropped_not_valid"), m["coverage"]))
# all OK frames empty (depth_off): not initialised, no ATE
trk_all_empty = os.path.join(out, "trk_all_empty.csv")
with open(trk_all_empty, "w") as f:
    f.write("timestamp,valid,state,quality_rgb,quality_depth,source,latency_ms,map_id,frame_epoch\n")
    for k in range(n):
        f.write(f"{t[k]:.6f},{int(k % 2 == 0)},{2 if k % 2 == 0 else 4},0,0,{'SLAM' if k % 2 == 0 else 'NONE'},10.0,"
                f"{k // 2},0\n")
write_tum(os.path.join(out, "origin.tum"), t[::2], np.zeros((len(t[::2]), 3)), np.tile([0, 0, 0, 1.0], (len(t[::2]), 1)))
m = run(out, "all_empty", os.path.join(out, "origin.tum"), trk_all_empty, "IMU_RGBD", ["--segment-by-map", "--drop-segment-first-frames", "0"])
check("all OK frames empty: coverage 0, not initialised, ATE None",
      m["coverage"] == 0 and not m["initialized"] and m["ate"] is None and m["n_ok_but_empty"] == len(t[::2]),
      (m["coverage"], m["initialized"], m["ate"], m["n_ok_but_empty"]))

# 16) --drop-segment-first-frames (default 1): first frame of each segment still in the OLD world frame
#     (T7 problem 1). With 3 segments and a wrong first frame in segments 2 and 3, default drop gives ATE~0.
mids = np.zeros(n, int)
eps = np.zeros(n, int)
eps[400:] = 1
eps[800:] = 2
Pf, Qf = P.copy(), Q.copy()
for e_, (ang, off) in {1: ([0, 0, 60], [2, 0, 0]), 2: ([0, 0, -40], [0, 3, 0])}.items():
    k = eps == e_
    Rk = R.from_euler("xyz", ang, degrees=True)
    Pf[k] = Rk.apply(P[k]) + np.array(off)
    Qf[k] = (Rk * R.from_quat(Q[k])).as_quat()
for first, prev_ep in [(400, 0), (800, 1)]:  # event frame keeps the previous epoch's frame
    Pf[first], Qf[first] = (P[first], Q[first]) if prev_ep == 0 else (Pf[first - 1] + (Pf[first - 1] - Pf[first - 2]), Qf[first - 1])
write_tum(os.path.join(out, "drop.tum"), t, Pf, Qf)
trkf = os.path.join(out, "trk_drop.csv")
write_trk_epoch(trkf, t, mids, eps)
os.makedirs(os.path.join(out, "droprun"), exist_ok=True)
trkf2 = os.path.join(out, "droprun", "tracking.csv")
os.replace(trkf, trkf2)
with open(os.path.join(out, "droprun", "run.log"), "w") as f:
    f.write("Loop detected!\nsomething\nLoop detected!\n")
m0 = run(out, "drop0", os.path.join(out, "drop.tum"), trkf2, "IMU_RGBD", ["--segment-by-map", "--drop-segment-first-frames", "0"])
m1 = run(out, "drop1", os.path.join(out, "drop.tum"), trkf2, "IMU_RGBD", ["--segment-by-map"])
check("no drop: wrong event frames inflate ATE", m0["ate"]["rmse"] > 0.05 and m0["n_dropped_segment_first"] == 0,
      (m0["ate"]["rmse"], m0["n_dropped_segment_first"]))
check("default drop 1: ATE~0, 3 frames dropped, n = n_valid - 3",
      m1["drop_segment_first_frames"] == 1 and m1["n_dropped_segment_first"] == 3 and m1["ate"]["rmse"] < 1e-4
      and m1["ate"]["n"] == n - 3 and m1["coverage"] == 1.0,
      (m1["drop_segment_first_frames"], m1["n_dropped_segment_first"], m1["ate"]["rmse"], m1["ate"]["n"]))
check("loop_closure from run.log", m1["loop_closure"] is True and m1["n_loop_events"] == 2,
      (m1["loop_closure"], m1["n_loop_events"]))
check("no run.log -> loop fields None", m["n_loop_events"] is None if "n_loop_events" in m else False,
      m.get("n_loop_events"))

# 17) truncated last line in tracking.csv (crashed run) must not crash; row counted
trkt = os.path.join(out, "trk_trunc.csv")
with open(trk_all) as f:
    lines = f.read().splitlines()
with open(trkt, "w") as f:
    f.write("\n".join(lines) + "\n" + lines[-1].split(",")[0] + ",1,2,329")
mt = run(out, "trunc", os.path.join(out, "id.tum"), trkt, "IMU_RGBD", [])
check("truncated row dropped and counted", mt["n_tracking_rows_truncated"] == 1 and mt["n_frames"] == n,
      (mt["n_tracking_rows_truncated"], mt["n_frames"]))

# 18) truncated last line in the estimated .tum (crashed run) must not fail the evaluation
estt = os.path.join(out, "id_trunc.tum")
with open(os.path.join(out, "id.tum")) as f:
    lines = f.read().splitlines()
with open(estt, "w") as f:
    f.write("\n".join(lines) + "\n" + lines[-1].rsplit(" ", 5)[0] + " 1.0 2.0\n")
me = run(out, "est_trunc", estt, trk_all, "IMU_RGBD")
check("truncated est row dropped, ATE still computed",
      me["n_truncated_lines"] == 1 and me["n_est_poses"] == n and me["ate"]["rmse"] < 1e-5,
      (me["n_truncated_lines"], me["n_est_poses"], me["ate"]["rmse"]))

# 19) a malformed row in the middle of the .tum must still be an error (only the tail may be truncated)
estm = os.path.join(out, "id_mid.tum")
with open(os.path.join(out, "id.tum")) as f:
    lines = f.read().splitlines()
lines.insert(n // 2, "1.0 2.0 3.0")
with open(estm, "w") as f:
    f.write("\n".join(lines) + "\n")
rm = subprocess.run([sys.executable, EVAL, "--est", estm, "--gt", GT, "--tracking", trk_all, "--mode", "IMU_RGBD",
                     "--out", os.path.join(out, "est_mid.json")], stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
check("malformed middle est row rejected", rm.returncode != 0, rm.returncode)

print("ALL PASS" if not fails else f"{len(fails)} FAILED: {fails}")
sys.exit(1 if fails else 0)
