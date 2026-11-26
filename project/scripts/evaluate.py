#!/usr/bin/env python
"""Evaluate one ORB-SLAM3 run against ground truth and write metrics.json.

Usage:
    python scripts/evaluate.py --est RUN/trajectory_est.tum --gt SEQ/trajectory_gt.tum \
        --tracking RUN/tracking.csv --mode IMU_RGBD --out RUN/metrics.json \
        [--align auto|se3|sim3] [--rpe-delta 1.0] [--seed 0] [--sequence NAME] [--config FILE] \
        [--fault-intervals faults.csv] [--recover-frames 3] [--recover-err 0.5]

Definitions (PROJECT_PLAN 3.2):
  * Pairing: est and gt poses are matched by timestamp (|dt| <= --max-dt). ATE/RPE use paired poses only.
  * Alignment: Umeyama on all paired poses; --align auto = sim3 for MONOCULAR, se3 for every other mode.
  * ATE: RMSE/mean/median/max of translation error after alignment (m).
  * RPE: for each paired pose i and the paired pose j with t_j - t_i = --rpe-delta (s, within --max-dt),
    error of the relative motion; translation (m) and rotation (deg). Scale is corrected first for sim3.
  * Valid frame: tracking.csv valid == 1 (state OK) AND quality_rgb (tracked map
    points) >= --min-tracked-points (default 1). ORB-SLAM3 reports OK for frames that "initialise" an empty
    map (0 points, pose at the origin), e.g. RGB-D without any depth; those are counted as
    n_ok_but_empty and treated as not valid everywhere (coverage, lost events, recovery, ATE pairing).
    The original tracking.csv is not modified.
  * Truncated output: a run killed by a signal can end mid-line in tracking.csv and in the estimated .tum.
    Such rows are dropped and counted (n_tracking_rows_truncated, n_truncated_lines); malformed .tum rows
    are only tolerated at the end of the file. Rows written before the crash are still evaluated.
  * Coverage: valid frames / all frames; paired_coverage: paired est poses / gt poses.
  * Lost: a frame with valid == 0 after the first valid frame. A maximal run of lost frames is one event.
    Frames before the first valid frame count as initialisation delay, not as a lost event.
  * Recovery: a lost event recovers at the first frame that starts --recover-frames consecutive valid
    frames (and, if --recover-err is set, whose aligned position errors are all <= that value).
    recovery_time_s = t(recovered frame) - t(first lost frame); unrecovered events get null, never 0.
  * --fault-intervals (CSV with columns start,end in seconds): additionally reports, per interval, the
    time from the fault end (observation restored) to the first reliable frame at or after it, the
    PROJECT_PLAN definition of recovery time.
  * Frames with source == IMU (IMU prediction during RECENTLY_LOST) are counted separately and are never
    treated as valid.
  * --segment-by-map (for the online trajectory, whose frame changes after a map reset): paired poses are
    split into maximal time-ordered runs of the same tracking.csv map_id; each run is aligned on its own
    (same se3/sim3 choice). Runs with < --min-segment-poses poses are listed but not aligned and excluded
    from ATE/RPE. Overall ATE/RPE pool the errors of all aligned runs, i.e. frame-weighted. RPE pairs never
    cross a run boundary. Estimated poses whose frame has no map_id are counted in n_paired_no_map_id.
    If tracking.csv also has a frame_epoch column (incremented by the runner whenever the world frame
    changes inside a map: IMU init, VIBA1/2, scale refinement, merge), runs are split on
    (map_id, frame_epoch) instead. More segments make ATE more optimistic, so n_segments and
    segment_frames must always be reported next to ATE; n_valid_frames_epoch0 counts valid frames
    before the first world-frame change (for IMU modes: before IMU initialisation).
  * --drop-segment-first-frames N (default 1): with --segment-by-map, the first N paired poses of each
    segment are excluded from alignment, ATE and RPE (event frames can still be in the old world frame);
    counted in n_dropped_segment_first. Coverage, lost events and recovery are not affected.
  * loop_closure / n_loop_events: from run.log next to tracking.csv ("Loop detected").
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
from evo.core import lie_algebra as lie
from evo.core.geometry import GeometryException
from evo.core.trajectory import PoseTrajectory3D


def load_tracking(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    # A crashed run (e.g. ORB-SLAM3 abort) can leave a partially written last line: drop rows with missing fields.
    header_len = len(rows[0]) if rows else 0
    complete = [r for r in rows if None not in r.values() and len(r) == header_len]
    load_tracking.n_truncated = len(rows) - len(complete)
    rows = complete
    t = np.array([float(r["timestamp"]) for r in rows])
    ok = np.array([int(float(r["valid"])) for r in rows], dtype=bool)
    pts = np.array([float(r["quality_rgb"]) if r.get("quality_rgb") not in (None, "") else np.nan for r in rows])
    source = [r.get("source", "") for r in rows]
    state = [r.get("state", "") for r in rows]
    lat = np.array([float(r["latency_ms"]) if r.get("latency_ms") not in (None, "") else np.nan for r in rows])
    has_map = bool(rows) and "map_id" in rows[0]
    has_epoch = bool(rows) and "frame_epoch" in rows[0]
    map_id = [r["map_id"] if has_map and r["map_id"] not in ("", "-1") else None for r in rows]
    epoch = [int(float(r["frame_epoch"])) if has_epoch and r["frame_epoch"] not in ("", "-1") else None
             for r in rows]
    return t, ok, pts, source, state, lat, map_id, has_map, epoch, has_epoch


def load_tum(path):
    """Read a TUM trajectory, dropping malformed rows at the end of the file only.

    A crashed ORB-SLAM3 (signal, exit > 128) can leave a partially written last line in a .tum file even
    though the rows before it are fine; evo's reader rejects the whole file in that case. A malformed row
    anywhere else is still an error. Returns None for a file with no usable row; the number of dropped
    lines is stored in load_tum.n_dropped.
    """
    with open(path, errors="replace") as f:
        lines = [ln.split() for ln in f if ln.strip() and not ln.lstrip().startswith("#")]
    good = []
    for k, p in enumerate(lines):
        try:
            if len(p) != 8:
                raise ValueError
            good.append([float(x) for x in p])
        except ValueError:
            if any(len(q) == 8 for q in lines[k + 1:]):
                raise SystemExit(f"{path}:{k + 1}: malformed TUM row before the last line")
    load_tum.n_dropped = len(lines) - len(good)
    if not good:
        return None
    mat = np.array(good)
    quat = np.roll(mat[:, 4:8], 1, axis=1)  # file order qx qy qz qw -> wxyz
    return PoseTrajectory3D(mat[:, 1:4], quat, mat[:, 0], name=path)


def associate(t_est, t_gt, max_dt):
    """Return index pairs (i_est, i_gt), nearest gt timestamp, one-to-one."""
    order = np.argsort(t_gt)
    tg = t_gt[order]
    pos = np.clip(np.searchsorted(tg, t_est), 1, len(tg) - 1)
    left, right = tg[pos - 1], tg[pos]
    pos = np.where(np.abs(t_est - left) <= np.abs(t_est - right), pos - 1, pos)
    ok = np.abs(tg[pos] - t_est) <= max_dt
    ie, ig = np.nonzero(ok)[0], order[pos[ok]]
    _, first = np.unique(ig, return_index=True)  # drop duplicate gt matches
    return ie[first], ig[first]


def stats(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return None
    return {"rmse": float(np.sqrt(np.mean(x ** 2))), "mean": float(np.mean(x)), "median": float(np.median(x)),
            "std": float(np.std(x)), "max": float(np.max(x)), "n": int(x.size)}


def rpe(est_poses, gt_poses, t, delta, max_dt):
    te, re = [], []
    j = 0
    for i in range(len(t)):
        target = t[i] + delta
        j = max(j, i + 1)
        while j < len(t) and t[j] < target - max_dt:
            j += 1
        if j >= len(t) or abs(t[j] - target) > max_dt:
            continue
        d_est = lie.relative_se3(est_poses[i], est_poses[j])
        d_gt = lie.relative_se3(gt_poses[i], gt_poses[j])
        e = lie.relative_se3(d_gt, d_est)
        te.append(np.linalg.norm(e[:3, 3]))
        re.append(math.degrees(lie.so3_log_angle(e[:3, :3])))
    return te, re


def align_segment(est, gt, ie, ig, align, a):
    """Umeyama-align paired poses est[ie] to gt[ig]; return errors and alignment."""
    e = PoseTrajectory3D(poses_se3=[est.poses_se3[i] for i in ie], timestamps=est.timestamps[ie])
    g = PoseTrajectory3D(poses_se3=[gt.poses_se3[i] for i in ig], timestamps=gt.timestamps[ig])
    r, tr, s = e.align(g, correct_scale=(align == "sim3"))
    pe = np.linalg.norm(e.positions_xyz - g.positions_xyz, axis=1)
    te, re = rpe(e.poses_se3, g.poses_se3, g.timestamps, a.rpe_delta, a.max_dt)
    return {"pe": pe, "te": te, "re": re, "scale": float(s),
            "align_rotation": np.asarray(r).tolist(), "align_translation": np.asarray(tr).tolist(),
            "err_by_t": {round(float(t), 6): float(x) for t, x in zip(g.timestamps, pe)},
            "path_length_gt_m": float(np.sum(np.linalg.norm(np.diff(g.positions_xyz, axis=0), axis=1)))}


def runs(mask):
    """Maximal runs of True as (start, end_exclusive)."""
    m = np.concatenate([[False], mask, [False]]).astype(int)
    d = np.diff(m)
    return list(zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--est", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--tracking", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mode", required=True, choices=["MONOCULAR", "IMU_MONOCULAR", "RGBD", "IMU_RGBD"])
    ap.add_argument("--align", default="auto", choices=["auto", "se3", "sim3"])
    ap.add_argument("--rpe-delta", type=float, default=1.0, help="seconds")
    ap.add_argument("--max-dt", type=float, default=0.005, help="timestamp association tolerance (s)")
    ap.add_argument("--recover-frames", type=int, default=3)
    ap.add_argument("--recover-err", type=float, default=None, help="m, aligned position error bound")
    ap.add_argument("--fault-intervals", default=None)
    ap.add_argument("--drop-segment-first-frames", type=int, default=1,
                    help="with --segment-by-map: drop the first N paired poses of every segment before alignment")
    ap.add_argument("--min-tracked-points", type=int, default=1,
                    help="valid = state OK and quality_rgb >= this (0 = state OK only)")
    ap.add_argument("--segment-by-map", action="store_true",
                    help="align each map segment separately (needs map_id column in tracking.csv)")
    ap.add_argument("--min-segment-poses", type=int, default=3,
                    help="segments with fewer paired poses are not aligned and not in ATE")
    ap.add_argument("--seed", default=None)
    ap.add_argument("--dataset", default="TartanAir V2")
    ap.add_argument("--sequence", default=None)
    ap.add_argument("--config", default=None)
    ap.add_argument("--extra", default=None, help="JSON string merged into metrics.json")
    a = ap.parse_args()

    align = a.align if a.align != "auto" else ("sim3" if a.mode == "MONOCULAR" else "se3")
    gt = load_tum(a.gt)
    if gt is None:
        ap.error(f"{a.gt}: no valid TUM rows")
    t_trk, ok_state, n_pts, source, state, lat, map_id, has_map, epoch, has_epoch = load_tracking(a.tracking)
    if np.isnan(n_pts).any() and a.min_tracked_points > 0:
        ap.error(f"{a.tracking}: quality_rgb missing on some rows; pass --min-tracked-points 0 to ignore it")
    valid = ok_state & ((n_pts >= a.min_tracked_points) | (a.min_tracked_points <= 0))
    if a.segment_by_map and not has_map:
        ap.error(f"--segment-by-map needs a map_id column in {a.tracking}")
    n_frames = len(t_trk)
    m = {
        "dataset": a.dataset,
        "sequence": a.sequence or os.path.basename(os.path.dirname(os.path.abspath(a.gt))),
        "mode": a.mode, "config": a.config, "seed": a.seed,
        "est_file": os.path.abspath(a.est), "gt_file": os.path.abspath(a.gt),
        "tracking_file": os.path.abspath(a.tracking),
        "alignment": align, "segment_by_map": a.segment_by_map,
        "rpe_delta_s": a.rpe_delta, "max_dt_s": a.max_dt,
        "min_tracked_points": a.min_tracked_points,
        "n_frames": n_frames, "n_gt": int(gt.num_poses),
    }

    # ---- tracking statistics --------------------------------------------------------------
    m["n_state_ok"] = int(ok_state.sum())
    m["n_ok_but_empty"] = int((ok_state & ~valid).sum())
    m["n_valid"] = int(valid.sum())
    m["coverage"] = float(valid.mean()) if n_frames else 0.0
    m["n_imu_predicted"] = int(sum(s == "IMU" for s in source))
    m["latency_ms_mean"] = float(np.nanmean(lat)) if n_frames else None
    m["n_tracking_rows_truncated"] = int(getattr(load_tracking, "n_truncated", 0))
    m["last_frame_t_s"] = float(t_trk[-1]) if n_frames else None
    # Loop closures (ORB-SLAM3 run.log "Loop detected"): the final trajectory of such runs is reshaped by the
    # loop correction, so final ATE must always be reported together with this flag.
    run_log = os.path.join(os.path.dirname(os.path.abspath(a.tracking)), "run.log")
    if os.path.isfile(run_log):
        with open(run_log, errors="replace") as fh:
            m["n_loop_events"] = sum(1 for line in fh if "Loop detected" in line)
        m["loop_closure"] = m["n_loop_events"] > 0
    else:
        m["n_loop_events"] = m["loop_closure"] = None
    first_valid = int(np.argmax(valid)) if valid.any() else None
    m["initialized"] = first_valid is not None
    m["init_delay_s"] = float(t_trk[first_valid] - t_trk[0]) if first_valid is not None else None

    # ---- ATE / RPE ----------------------------------------------------------------------------
    err_by_t = {}
    est = None
    m["n_truncated_lines"] = 0
    if os.path.isfile(a.est) and os.path.getsize(a.est) > 0:
        est = load_tum(a.est)
        m["n_truncated_lines"] = int(getattr(load_tum, "n_dropped", 0))
    if est is not None:
        valid_t = {round(float(x), 6) for x, v in zip(t_trk, valid) if v}
        keep = [i for i, x in enumerate(est.timestamps) if round(float(x), 6) in valid_t]
        m["n_est_poses"] = int(est.num_poses)
        m["n_est_dropped_not_valid"] = int(est.num_poses - len(keep))
        est = (PoseTrajectory3D(poses_se3=[est.poses_se3[i] for i in keep], timestamps=est.timestamps[keep])
               if keep else None)
    if est is None or est.num_poses < 3:
        m["n_paired"] = 0 if est is None else int(est.num_poses)
        m.setdefault("n_est_poses", 0)
        m["paired_coverage"] = 0.0
        m["ate"] = m["rpe_trans"] = m["rpe_rot_deg"] = None
        m["scale"] = None
        m["note"] = "fewer than 3 estimated poses; ATE/RPE not computed"
    else:
        ie, ig = associate(est.timestamps, gt.timestamps, a.max_dt)
        m["n_paired"] = int(len(ie))
        m["paired_coverage"] = float(len(ie) / gt.num_poses)
        if len(ie) < 3:
            m["ate"] = m["rpe_trans"] = m["rpe_rot_deg"] = None
            m["scale"] = None
            m["note"] = "fewer than 3 paired poses; ATE/RPE not computed"
        elif not a.segment_by_map:
            seg = align_segment(est, gt, ie, ig, align, a)
            for k in ["scale", "align_rotation", "align_translation"]:
                m[k] = seg[k]
            m["ate"] = stats(seg["pe"])
            m["rpe_trans"] = stats(seg["te"])
            m["rpe_rot_deg"] = stats(seg["re"])
            err_by_t = seg["err_by_t"]
            m["path_length_gt_m"] = seg["path_length_gt_m"]
        else:
            # Segment key: map_id, plus frame_epoch when present (world-frame changes inside one map,
            # e.g. IMU initialisation / VIBA1 / VIBA2 / scale refinement / map merge).
            m["segment_key"] = "map_id,frame_epoch" if has_epoch else "map_id"
            if has_epoch:
                ve = [e for e, v in zip(epoch, valid) if v and e is not None]
                m["n_valid_frames_epoch0"] = int(sum(e == 0 for e in ve))
                first_epoch = {}
                for mid, e, v in zip(map_id, epoch, valid):
                    if v and mid is not None and e is not None:
                        first_epoch.setdefault(mid, e)
                m["n_valid_frames_first_epoch_of_map"] = int(sum(
                    1 for mid, e, v in zip(map_id, epoch, valid) if v and mid is not None and e == first_epoch.get(mid)))
                m["n_epochs"] = len(set(ve))
            map_of_t = {round(float(t), 6): (mid if not has_epoch else (None if mid is None or e is None else f"{mid}:{e}"))
                        for t, mid, e, v in zip(t_trk, map_id, epoch, valid) if v}
            key = [map_of_t.get(round(float(t), 6)) for t in est.timestamps[ie]]
            order = np.argsort(est.timestamps[ie])
            ie, ig, key = ie[order], ig[order], [key[k] for k in order]
            groups = []  # maximal time-ordered runs of the same map_id
            for k, mid in enumerate(key):
                if mid is None:
                    continue
                if groups and groups[-1][0] == mid:
                    groups[-1][1].append(k)
                else:
                    groups.append((mid, [k]))
            m["n_paired_no_map_id"] = int(sum(mid is None for mid in key))
            segs, pe, te, re = [], [], [], []
            n_dropped = 0
            for si, (mid, ks) in enumerate(groups):
                d = {"segment": si, "map_id": mid, "n": len(ks),
                     "t_start": float(est.timestamps[ie[ks[0]]]), "t_end": float(est.timestamps[ie[ks[-1]]])}
                # About 1 in 5 frame_epoch event frames still carries the old world frame, so the
                # first frame(s) of each segment are dropped before alignment.
                drop = min(a.drop_segment_first_frames, len(ks))
                d["n_dropped_first"] = drop
                n_dropped += drop
                ks = ks[drop:]
                d["n_used"] = len(ks)
                seg = None
                if len(ks) < a.min_segment_poses:
                    d.update(aligned=False, ate=None, scale=None, reason="too_few_poses")
                else:
                    try:
                        seg = align_segment(est, gt, ie[ks], ig[ks], align, a)
                    except GeometryException:
                        # e.g. all poses of the segment identical (no motion): Umeyama undefined
                        d.update(aligned=False, ate=None, scale=None, reason="degenerate")
                if seg is not None:
                    d.update(aligned=True, ate=stats(seg["pe"]), rpe_trans=stats(seg["te"]),
                             rpe_rot_deg=stats(seg["re"]), scale=seg["scale"],
                             path_length_gt_m=seg["path_length_gt_m"])
                    pe += list(seg["pe"])
                    te += seg["te"]
                    re += seg["re"]
                    err_by_t.update(seg["err_by_t"])
                segs.append(d)
            m["segments"] = segs
            m["n_segments"] = len(segs)
            m["n_segments_aligned"] = int(sum(d["aligned"] for d in segs))
            m["n_segments_degenerate"] = int(sum(d.get("reason") == "degenerate" for d in segs))
            m["segment_frames"] = [d["n"] for d in segs]
            m["drop_segment_first_frames"] = a.drop_segment_first_frames
            m["n_dropped_segment_first"] = int(n_dropped)
            m["n_poses_in_aligned_segments"] = int(len(pe))
            m["aligned_coverage"] = float(len(pe) / gt.num_poses)
            # Pooled over all poses of aligned segments, i.e. frame-weighted:
            # rmse = sqrt(sum_i n_i * rmse_i^2 / sum_i n_i)
            m["ate"] = stats(pe)
            m["rpe_trans"] = stats(te)
            m["rpe_rot_deg"] = stats(re)
            m["scale"] = None
            if not pe:
                m["note"] = f"no map segment with >= {a.min_segment_poses} poses; ATE/RPE not computed"

    # ---- lost events & recovery -----------------------------------------------------------
    def reliable(k):
        if not valid[k]:
            return False
        if a.recover_err is None:
            return True
        x = err_by_t.get(round(float(t_trk[k]), 6))
        return x is not None and x <= a.recover_err

    def recovery_index(start):
        """First frame >= start that begins recover_frames consecutive reliable frames."""
        k = start
        while k + a.recover_frames <= n_frames:
            if all(reliable(k + q) for q in range(a.recover_frames)):
                return k
            k += 1
        return None

    lost = np.zeros(n_frames, dtype=bool)
    if first_valid is not None:
        lost[first_valid:] = ~valid[first_valid:]
    events = []
    for s0, s1 in runs(lost):
        k = recovery_index(s1) if s1 < n_frames else None
        events.append({
            "start_s": float(t_trk[s0]),
            "lost_frames": int(s1 - s0),
            "lost_duration_s": float((t_trk[s1] if s1 < n_frames else t_trk[-1] + (t_trk[-1] - t_trk[-2])) - t_trk[s0]),
            "imu_predicted_frames": int(sum(source[q] == "IMU" for q in range(s0, s1))),
            "recovered": k is not None,
            "recovery_time_s": float(t_trk[k] - t_trk[s0]) if k is not None else None,
        })
    m["lost_events"] = len(events)
    m["lost_frames"] = int(lost.sum())
    m["lost_rate"] = float(lost.sum() / n_frames) if n_frames else None
    rec = [e["recovery_time_s"] for e in events if e["recovered"]]
    m["n_unrecovered"] = int(sum(not e["recovered"] for e in events))
    m["recovery_time_s"] = stats(rec)
    m["events"] = events

    if a.fault_intervals:
        out = []
        with open(a.fault_intervals) as f:
            for r in csv.DictReader(f):
                t0, t1 = float(r["start"]), float(r["end"])
                k0 = int(np.searchsorted(t_trk, t1 - 1e-6))
                k = recovery_index(k0) if k0 < n_frames else None
                out.append({"fault_start_s": t0, "fault_end_s": t1,
                            "valid_frames_during_fault": int(valid[(t_trk >= t0) & (t_trk < t1)].sum()),
                            "recovered": k is not None,
                            "recovery_time_s": float(t_trk[k] - t1) if k is not None else None})
        m["fault_recovery"] = out

    if a.extra:
        m.update(json.loads(a.extra))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(m, f, indent=2)
    summary = {k: m[k] for k in ["sequence", "mode", "alignment", "n_paired", "coverage", "n_ok_but_empty",
                                  "lost_events", "n_unrecovered"]}
    summary["ate_rmse"] = m["ate"]["rmse"] if m.get("ate") else None
    summary["rpe_trans_rmse"] = m["rpe_trans"]["rmse"] if m.get("rpe_trans") else None
    summary["rpe_rot_deg_rmse"] = m["rpe_rot_deg"]["rmse"] if m.get("rpe_rot_deg") else None
    summary["scale"] = m.get("scale")
    if a.segment_by_map:
        summary["n_segments"] = m.get("n_segments")
        summary["n_segments_aligned"] = m.get("n_segments_aligned")
        summary["segment_key"] = m.get("segment_key")
        if "n_valid_frames_epoch0" in m:
            summary["n_valid_frames_epoch0"] = m["n_valid_frames_epoch0"]
    print(json.dumps(summary))


if __name__ == "__main__":
    sys.exit(main())
