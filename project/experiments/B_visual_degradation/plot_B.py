#!/usr/bin/env python3
"""B: summary table and figures from the per-run CSV of scripts/aggregate.py.

  table   per (sequence, fault, level, mode): mean/std/median over seeds of the key metrics, plus run and
          failure counts -> results/B/visual_stress.csv. The clean runs are the L0 row of every fault.
  curves  per sequence, one PNG per metric: 5 panels (faults) x level L0-L4, one line per mode,
          error bars = std over seeds (only groups with >= 2 values get a bar).
  case    one run directory: aligned online trajectory over GT (top view, one colour per segment)
          + tracking-state timeline + tracked map points. --auto picks the largest coverage drops.

Typical:
  env_data/bin/python scripts/aggregate.py results/B/runs --out results/B/visual_stress_runs.csv
  env_data/bin/python experiments/B_visual_degradation/plot_B.py all --runs-csv results/B/visual_stress_runs.csv \
      --runs-dir results/B/runs
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
FAULTS = ["gaussian_blur", "motion_blur", "brightness_contrast", "occlusion", "texture_erase"]
ZH_FONT = os.path.join(ROOT, "env_docs/share/fonts/noto-serif-cjk-sc/NotoSerifCJKsc-Regular.otf")
# User-visible text. Mode names and ATE/RPE stay English in both languages.
TXT = {
    "en": {
        "fault": {"gaussian_blur": "Gaussian blur", "motion_blur": "Motion blur",
                  "brightness_contrast": "Low light / contrast", "occlusion": "Occlusion",
                  "texture_erase": "Texture erase (image-level)"},
        "metric": {"coverage": "Valid-pose coverage (fraction of frames)",
                   "lost_events": "Lost events (count)",
                   "ate_online_rmse": "ATE RMSE, online (m)\nper-segment aligned, coverage >= 0.1",
                   "aligned_coverage": "Poses in aligned segments / GT poses",
                   "n_segments": "Tracking segments\n(map resets + frame changes)",
                   "n_unrecovered": "Unrecovered lost events (count)\nrecover-err 0.5 m"},
        "metric_short": {"coverage": "Valid-pose coverage", "lost_events": "Lost events",
                         "ate_online_rmse": "ATE (online)", "aligned_coverage": "Aligned coverage",
                         "n_segments": "Tracking segments", "n_unrecovered": "Unrecovered lost events"},
        "xlevel": "Severity level (L0 = clean)",
        "failed_marker": "fewer seeds than runs (failed or coverage < 0.1)",
        "initfail_marker": "no value: all seeds failed to initialise or coverage < 0.1",
        "curve_title": "B visual degradation - {m} vs severity  [{seq}]  mean +/- std over seeds",
        "state": ["OK (valid)", "RECENTLY_LOST", "LOST", "not initialised"],
        "gt": "ground truth", "est": "estimate (per segment)",
        "east": "East (m)", "north": "North (m)",
        "top_title": "Top view: online estimate, each segment aligned to GT\n"
                     "{n} segments with >= 3 poses (8 largest coloured, rest gray)",
        "state_title": "Tracking state per frame (black ticks at top = new segment)",
        "pts": "Tracked map points", "pts_title": "Tracked map points (quality_rgb)",
        "time": "Time since first frame (s)",
        "case_title": "{run}   status={st}   coverage={cov:.2f}   lost events={le}   segments={seg}   ATE(online)={ate}",
        "na": "n/a",
    },
    "zh": {
        "fault": {"gaussian_blur": "高斯模糊", "motion_blur": "运动模糊", "brightness_contrast": "低照度 / 低对比度",
                  "occlusion": "遮挡", "texture_erase": "纹理抹除（图像级近似）"},
        "metric": {"coverage": "有效位姿覆盖率（占全部帧的比例）",
                   "lost_events": "失跟事件数（次）",
                   "ate_online_rmse": "ATE RMSE（online，m）\n按段对齐，仅覆盖率 ≥ 0.1",
                   "aligned_coverage": "已对齐段内位姿数 / GT 位姿数",
                   "n_segments": "跟踪段数（地图重置 + 坐标系变化）",
                   "n_unrecovered": "未恢复的失跟事件数（次；recover-err 0.5 m）"},
        "metric_short": {"coverage": "有效位姿覆盖率", "lost_events": "失跟事件数",
                         "ate_online_rmse": "ATE（online）", "aligned_coverage": "已对齐覆盖率",
                         "n_segments": "跟踪段数", "n_unrecovered": "未恢复失跟事件数"},
        "xlevel": "退化强度（L0 = 无扰动）",
        "failed_marker": "有效种子少于运行数（失败或覆盖率 < 0.1）",
        "initfail_marker": "无值：全部种子初始化失败或覆盖率 < 0.1",
        "curve_title": "B 视觉退化：{m}随强度的变化  [{seq}]  种子间均值 ± 标准差",
        "state": ["OK（有效）", "RECENTLY_LOST", "LOST", "未初始化"],
        "gt": "真值轨迹", "est": "估计轨迹（按段对齐）",
        "east": "东向（m）", "north": "北向（m）",
        "top_title": "俯视图：online 估计轨迹，每段分别对齐到真值\n"
                     "共 {n} 段（≥3 个位姿）；最长的 8 段着色，其余为灰色",
        "state_title": "逐帧跟踪状态（顶部黑色刻度 = 新段开始）",
        "pts": "跟踪到的地图点数", "pts_title": "跟踪到的地图点数（quality_rgb）",
        "time": "距首帧时间（s）",
        "case_title": "{run}   状态={st}   覆盖率={cov:.2f}   失跟事件={le}   段数={seg}   ATE(online)={ate}",
        "na": "无",
    },
}
L = TXT["en"]


def set_lang(lang):
    """Select label language; for zh register the static CJK font (matplotlib cannot read the -VF one)."""
    global L
    L = TXT[lang]
    if lang == "zh":
        from matplotlib import font_manager as fm
        if not os.path.isfile(ZH_FONT):
            sys.exit(f"Chinese font not found: {ZH_FONT} (use --lang en)")
        fm.fontManager.addfont(ZH_FONT)
        matplotlib.rcParams["font.family"] = fm.FontProperties(fname=ZH_FONT).get_name()
    matplotlib.rcParams["axes.unicode_minus"] = False


LEVELS = ["L0", "L1", "L2", "L3", "L4"]
MODES = ["MONOCULAR", "IMU_MONOCULAR", "IMU_RGBD"]
# Categorical slots 1-3 (validated all-pairs, light surface); aqua < 3:1 contrast -> markers + direct labels.
MODE_STYLE = {
    "MONOCULAR": dict(color="#2a78d6", marker="o", label="Mono"),
    "IMU_MONOCULAR": dict(color="#eb6834", marker="s", label="Mono+IMU"),
    "IMU_RGBD": dict(color="#1baf7a", marker="^", label="RGB-D+IMU"),
}
STATE_STYLE = [  # tracking state -> (index into L["state"], colour); status palette, never series colours
    ((2,), 0, "#0ca30c"),
    ((3,), 1, "#fab219"),
    ((4,), 2, "#d03b3b"),
    ((-1, 0, 1), 3, "#b9b8b3"),
]
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"

# Coverage is a real value (0) for runs that never initialise; every other metric is undefined for them
# (e.g. evaluate.py counts lost events only after the first valid frame, so they would show 0 events).
COUNT_FAILED_AS_VALUE = {"coverage"}
# ATE/RPE from a handful of frames is not an accuracy figure (e.g. gaussian_blur L4 IMU_RGBD, 17 poses).
MIN_COVERAGE_FOR_ERROR = 0.1
ERROR_METRICS = {"ate_online_rmse", "rpe_trans_online_rmse", "rpe_rot_online_deg_rmse", "ate_final_rmse"}

# metric column in aggregate.py CSV -> file stem (axis labels in TXT)
METRICS = {"coverage": "coverage", "lost_events": "lost_events", "ate_online_rmse": "ate_online",
           "aligned_coverage": "aligned_coverage", "n_segments": "segments", "n_unrecovered": "unrecovered"}
TABLE_METRICS = ["coverage", "lost_events", "lost_rate", "n_unrecovered", "recovery_time_median_s",
                 "init_delay_s", "n_segments", "n_segments_aligned", "aligned_coverage", "ate_online_rmse",
                 "rpe_trans_online_rmse", "rpe_rot_online_deg_rmse", "ate_final_rmse", "final_paired_coverage",
                 "n_imu_predicted", "n_loops"]  # n_loops: report next to ate_final (loop closure changes it)


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=9)


def num(v):
    if v in (None, "") or isinstance(v, bool):
        return None
    try:
        x = float(v)
    except ValueError:
        return None
    return x if math.isfinite(x) else None


# ---------------------------------------------------------------- table

def load_runs(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"no runs in {path}")
    out = []
    for r in rows:
        if r.get("task") not in ("B", None, ""):
            continue
        r = dict(r)
        r["level"] = "L0" if r["fault"] == "clean" else r["level"]
        out.append(r)
    return out


def expand_clean(rows):
    """Each clean run becomes the L0 point of every fault (same seed, same IMU noise)."""
    faults = sorted({r["fault"] for r in rows if r["fault"] != "clean"}, key=lambda f: FAULTS.index(f)
                    if f in FAULTS else 99)
    out = [r for r in rows if r["fault"] != "clean"]
    for r in rows:
        if r["fault"] == "clean":
            out += [dict(r, fault=f) for f in faults]
    return out, faults


def group(rows):
    g = {}
    for r in rows:
        g.setdefault((r["base_sequence"], r["fault"], r["level"], r["mode"]), []).append(r)
    return g


def usable(rs, col):
    """Runs whose value of col is meaningful (see COUNT_FAILED_AS_VALUE, MIN_COVERAGE_FOR_ERROR)."""
    if col not in COUNT_FAILED_AS_VALUE:
        rs = [r for r in rs if r.get("status") == "ok"]
    if col in ERROR_METRICS:
        rs = [r for r in rs if (num(r.get("coverage")) or 0.0) >= MIN_COVERAGE_FOR_ERROR]
    return rs


def summarise(rs, col):
    vals = [num(r.get(col)) for r in rs]
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return {"mean": statistics.fmean(vals), "std": statistics.stdev(vals) if len(vals) > 1 else 0.0,
            "median": statistics.median(vals), "n": len(vals)}


def cmd_table(rows, out):
    rows, _ = expand_clean(rows)
    g = group(rows)
    header = ["base_sequence", "fault", "level", "mode", "seeds", "n_runs", "n_ok", "n_init_failed",
              "n_other_failed", "n_loop_runs"]
    for c in TABLE_METRICS:
        header += [f"{c}_mean", f"{c}_std", f"{c}_median", f"{c}_n"]
    # ATE over all ok runs regardless of coverage, so the filtered value can be checked against it
    header += ["ate_online_rmse_allok_mean", "ate_online_rmse_allok_n"]
    lines = []
    key = lambda k: (k[0], FAULTS.index(k[1]) if k[1] in FAULTS else 99, k[2],
                     MODES.index(k[3]) if k[3] in MODES else 99)
    for k in sorted(g, key=key):
        rs = g[k]
        o = dict(zip(header[:4], k))
        o["seeds"] = " ".join(sorted(str(r["seed"]) for r in rs))
        o["n_runs"] = len(rs)
        o["n_ok"] = sum(r["status"] == "ok" for r in rs)
        o["n_init_failed"] = sum(r["status"] == "init_failed" for r in rs)
        o["n_other_failed"] = o["n_runs"] - o["n_ok"] - o["n_init_failed"]
        o["n_loop_runs"] = sum(str(r.get("loop_detected")).lower() == "true" for r in rs)
        for c in TABLE_METRICS:
            s = summarise(usable(rs, c), c)
            for sub in ("mean", "std", "median", "n"):
                o[f"{c}_{sub}"] = "" if s is None else (round(s[sub], 6) if sub != "n" else s[sub])
        s = summarise([r for r in rs if r.get("status") == "ok"], "ate_online_rmse")
        o["ate_online_rmse_allok_mean"] = "" if s is None else round(s["mean"], 6)
        o["ate_online_rmse_allok_n"] = "" if s is None else s["n"]
        lines.append(o)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(lines)
    print(f"[table] {out}  ({len(lines)} groups)")


# ---------------------------------------------------------------- curves

def cmd_curves(rows, out_dir, metrics, log_ate):
    rows, faults = expand_clean(rows)
    g = group(rows)
    seqs = sorted({r["base_sequence"] for r in rows})
    modes = [m for m in MODES if any(r["mode"] == m for r in rows)]
    os.makedirs(out_dir, exist_ok=True)
    for seq in seqs:
        for col in metrics:
            ylabel, stem = L["metric"][col], METRICS[col]
            fig, axes = plt.subplots(1, len(faults), figsize=(max(3.3 * len(faults), 13.0), 4.0), sharey=True,
                                     facecolor=SURFACE)
            axes = np.atleast_1d(axes)
            any_data = False
            for ax, fault in zip(axes, faults):
                style_axes(ax)
                xs_all = range(len(LEVELS))
                for mi, mode in enumerate(modes):
                    st = MODE_STYLE[mode]
                    xs, ys, es, ns, nr = [], [], [], [], []
                    for li, lv in enumerate(LEVELS):
                        rs = g.get((seq, fault, lv, mode), [])
                        dx = li + (mi - (len(modes) - 1) / 2) * 0.08  # dodge so bars do not overlap
                        s = summarise(usable(rs, col), col) if rs else None
                        if s is None:
                            if rs:  # runs exist but none has a usable value: mark instead of a silent gap
                                ax.plot([dx], [0.04], marker="x", markersize=8, markeredgewidth=2,
                                        color=st["color"], transform=ax.get_xaxis_transform(), zorder=4,
                                        clip_on=False)
                            continue
                        xs.append(dx)
                        ys.append(s["mean"])
                        es.append(s["std"] if s["n"] > 1 else 0.0)
                        ns.append(s["n"])
                        nr.append(len(rs))
                    if not xs:
                        continue
                    any_data = True
                    ax.errorbar(xs, ys, yerr=es, color=st["color"], marker=st["marker"], markersize=6,
                                linewidth=2, capsize=3, elinewidth=1.2, markeredgecolor=SURFACE,
                                markeredgewidth=1.0, zorder=3)
                    # hollow marker where fewer values than runs (failed runs carry no value)
                    for x, y, n, r in zip(xs, ys, ns, nr):
                        if n < r:
                            ax.plot([x], [y], marker=st["marker"], markersize=10, markerfacecolor="none",
                                    markeredgecolor=INK, markeredgewidth=1.0, zorder=4)
                ax.set_xticks(list(xs_all))
                ax.set_xticklabels(LEVELS)
                ax.set_xlim(-0.4, len(LEVELS) - 0.6)
                ax.set_title(L["fault"].get(fault, fault), fontsize=10, color=INK)
                ax.set_xlabel(L["xlevel"], fontsize=9, color=INK2)
                if col == "coverage" or col == "aligned_coverage":
                    ax.set_ylim(-0.03, 1.03)
                if col == "ate_online_rmse" and log_ate:
                    ax.set_yscale("log")
            if not any_data:
                plt.close(fig)
                print(f"[curves] skip {seq} {col}: no data")
                continue
            axes[0].set_ylabel(ylabel, fontsize=9, color=INK2)
            handles = [Line2D([], [], color=MODE_STYLE[m]["color"], marker=MODE_STYLE[m]["marker"],
                              linewidth=2, label=MODE_STYLE[m]["label"]) for m in modes]
            handles.append(Line2D([], [], linestyle="none", marker="o", markersize=9, markerfacecolor="none",
                                  markeredgecolor=INK, label=L["failed_marker"]))
            if col not in COUNT_FAILED_AS_VALUE:
                handles.append(Line2D([], [], linestyle="none", marker="x", markersize=8, markeredgewidth=2,
                                      color=INK2, label=L["initfail_marker"]))
            fig.legend(handles=handles, loc="upper center", ncol=min(len(handles), 3), frameon=False, fontsize=9,
                       bbox_to_anchor=(0.5, 0.995))
            fig.suptitle(L["curve_title"].format(m=L["metric_short"][col], seq=seq), fontsize=11, color=INK,
                         y=0.86)
            fig.tight_layout(rect=(0, 0, 1, 0.82))
            p = os.path.join(out_dir, f"{stem}_vs_level_{seq}.png")
            fig.savefig(p, dpi=150, facecolor=SURFACE)
            plt.close(fig)
            print(f"[curves] {p}")


# ---------------------------------------------------------------- failure case

def read_tum(path):
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        return np.zeros(0), np.zeros((0, 3))
    a = np.loadtxt(path, ndmin=2)
    return a[:, 0], a[:, 1:4]


def read_tracking(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["timestamp"]) for r in rows])
    state = np.array([int(float(r["state"])) for r in rows])
    q = np.array([num(r.get("quality_rgb")) or 0.0 for r in rows])
    seg = []
    for r in rows:
        mid = r.get("map_id", "-1")
        ep = r.get("frame_epoch", "")
        seg.append(None if mid in ("", "-1") or r["valid"] in ("0", "0.0") else (mid, ep))
    return t, state, q, seg


def umeyama(src, dst, with_scale):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    u, d, vt = np.linalg.svd(cov)
    s_ = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s_[2, 2] = -1
    r = u @ s_ @ vt
    c = (d * np.diag(s_)).sum() / xs.var(0).sum() if with_scale else 1.0
    return c, r, mu_d - c * r @ mu_s


def cmd_case(run_dir, out, gt_override=None):
    mj = json.load(open(os.path.join(run_dir, "metrics.json")))
    mr = mj.get("matrix_run") or {}
    seq_dir = mr.get("sequence_dir") or ""
    gt_path = gt_override or os.path.join(seq_dir, "trajectory_gt.tum")
    if not os.path.isfile(gt_path):
        base = mr.get("base_sequence") or mj.get("sequence", "").split("__")[0]
        gt_path = os.path.join(ROOT, "data", "seqs", base, "trajectory_gt.tum")
    mode = mj.get("mode", mr.get("mode", ""))
    t_gt, p_gt = read_tum(gt_path)
    t_est, p_est = read_tum(os.path.join(run_dir, "trajectory_online.tum"))
    t_trk, state, q, seg = read_tracking(os.path.join(run_dir, "tracking.csv"))
    gt_of = {round(t, 6): i for i, t in enumerate(t_gt)}
    seg_of = {round(t, 6): s for t, s in zip(t_trk, seg)}

    # group est poses by segment, align each with >= 3 poses (sim3 for MONOCULAR, as evaluate.py)
    segs = {}
    order = []
    for i, t in enumerate(t_est):
        k = seg_of.get(round(t, 6))
        j = gt_of.get(round(t, 6))
        if k is None or j is None:
            continue
        if k not in segs:
            segs[k] = []
            order.append(k)
        segs[k].append((i, j))
    fig = plt.figure(figsize=(12, 6.4), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.35], height_ratios=[1, 1], wspace=0.22, hspace=0.35)
    ax = fig.add_subplot(gs[:, 0])
    style_axes(ax)
    ax.plot(p_gt[:, 1], p_gt[:, 0], color=INK, linewidth=1.2, label=L["gt"], zorder=2)
    seg_cols = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
    big = sorted(order, key=lambda k: -len(segs[k]))
    n_plotted = 0
    for rank, k in enumerate(big):
        pairs = segs[k]
        if len(pairs) < 3:
            continue
        ie, ig = np.array(pairs).T
        c, r, tr = umeyama(p_est[ie], p_gt[ig], mode == "MONOCULAR")
        al = (c * (r @ p_est[ie].T)).T + tr
        col = seg_cols[rank] if rank < len(seg_cols) else "#8a8984"  # beyond 8 segments: one gray
        ax.plot(al[:, 1], al[:, 0], color=col, linewidth=2, zorder=3)
        n_plotted += 1
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel(L["east"], fontsize=9, color=INK2)
    ax.set_ylabel(L["north"], fontsize=9, color=INK2)
    ax.set_title(L["top_title"].format(n=n_plotted), fontsize=9, color=INK)
    ax.legend(handles=[Line2D([], [], color=INK, linewidth=1.2, label=L["gt"]),
                       Line2D([], [], color=seg_cols[0], linewidth=2, label=L["est"])],
              frameon=False, fontsize=8, loc="best")

    t0 = t_trk[0] if len(t_trk) else 0.0
    tt = t_trk - t0
    dt = float(np.median(np.diff(tt))) if len(tt) > 1 else 0.1
    a1 = fig.add_subplot(gs[0, 1])
    style_axes(a1)
    a1.grid(False)
    for codes, _, col in STATE_STYLE:
        m = np.isin(state, codes)
        a1.bar(tt[m], np.ones(m.sum()), width=dt, color=col, align="edge", linewidth=0)
    # segment boundaries
    prev = None
    for x, s in zip(tt, seg):
        if s is not None and prev is not None and s != prev:
            a1.axvline(x, color=INK, linewidth=0.6, ymin=0.9, ymax=1.0)
        if s is not None:
            prev = s
    fault_win(a1, run_dir, t0)
    a1.set_yticks([])
    a1.set_xlim(tt[0], tt[-1] + dt)
    a1.set_title(L["state_title"], fontsize=9, color=INK)
    a1.legend(handles=[Patch(color=c, label=L["state"][k]) for _, k, c in STATE_STYLE], ncol=4, frameon=False, fontsize=8,
              loc="upper center", bbox_to_anchor=(0.5, -0.12))

    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    style_axes(a2)
    a2.plot(tt, q, color="#2a78d6", linewidth=1.5)
    fault_win(a2, run_dir, t0)
    a2.set_ylabel(L["pts"], fontsize=9, color=INK2)
    a2.set_xlabel(L["time"], fontsize=9, color=INK2)
    a2.set_title(L["pts_title"], fontsize=9, color=INK)

    ate = (mj.get("ate") or {}).get("rmse")
    fig.suptitle(L["case_title"].format(run=os.path.basename(os.path.normpath(run_dir)),
                                        st=mj.get("status", "?"), cov=mj.get("coverage", float("nan")),
                                        le=mj.get("lost_events"), seg=mj.get("n_segments"),
                                        ate=f"{ate:.3f} m" if ate is not None else L["na"]),
                 fontsize=10, color=INK)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=150, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)
    print(f"[case] {out}")


def fault_win(ax, run_dir, t0):
    p = os.path.join(run_dir, "fault_meta.json")
    if not os.path.isfile(p):
        return
    m = json.load(open(p))
    if m.get("window_first_ts") is not None and (m.get("window") or {}).get("duration_s") is not None:
        ax.axvspan(m["window_first_ts"] - t0, m["window_last_ts"] - t0, color=INK, alpha=0.08, linewidth=0)


def auto_cases(rows, runs_dir, n, seq_pref):
    """Groups with the largest drop in mean coverage from L0, IMU modes first; returns seed-0 run dirs."""
    rows_e, _ = expand_clean(rows)
    g = group(rows_e)
    cand = []
    for (seq, fault, lv, mode), rs in g.items():
        if lv == "L0" or fault == "clean":
            continue
        base = summarise(g.get((seq, fault, "L0", mode), []), "coverage")
        cur = summarise(rs, "coverage")
        if base is None or cur is None:
            continue
        drop = base["mean"] - cur["mean"]
        cand.append((seq == seq_pref, mode != "MONOCULAR", drop, seq, fault, lv, mode, rs))
    cand.sort(key=lambda c: (c[0], c[1], c[2]), reverse=True)
    picked, seen = [], set()
    for c in cand:
        if (c[4], c[6]) in seen:  # at most one per (fault, mode)
            continue
        rs = sorted(c[7], key=lambda r: str(r["seed"]))
        rd = os.path.join(runs_dir, rs[0]["run"])
        if os.path.isfile(os.path.join(rd, "tracking.csv")):
            picked.append((rd, c[2]))
            seen.add((c[4], c[6]))
        if len(picked) >= n:
            break
    return picked


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["table", "curves", "case", "all"])
    ap.add_argument("--runs-csv", default=os.path.join(ROOT, "results/B/visual_stress_runs.csv"),
                    help="per-run CSV from scripts/aggregate.py")
    ap.add_argument("--runs-dir", default=os.path.join(ROOT, "results/B/runs"))
    ap.add_argument("--table-out", default=os.path.join(ROOT, "results/B/visual_stress.csv"))
    ap.add_argument("--fig-dir", default=os.path.join(ROOT, "results/B/figures"))
    ap.add_argument("--metrics", default="coverage,lost_events,n_unrecovered,ate_online_rmse,aligned_coverage,n_segments")
    ap.add_argument("--linear-ate", action="store_true", help="linear instead of log ATE axis")
    ap.add_argument("--run", action="append", default=[], help="case: run directory (repeatable)")
    ap.add_argument("--auto", type=int, default=2, help="case/all: auto-pick this many failure cases")
    ap.add_argument("--prefer-seq", default="office_easy_p004")
    ap.add_argument("--lang", choices=["zh", "en"], default="en", help="label language (default en)")
    a = ap.parse_args()
    set_lang(a.lang)

    metrics = [m for m in a.metrics.split(",") if m]
    for m in metrics:
        if m not in METRICS:
            sys.exit(f"unknown metric {m}; choose from {list(METRICS)}")
    rows = load_runs(a.runs_csv) if (a.cmd != "case" or a.auto and not a.run) else []
    if a.cmd in ("table", "all"):
        cmd_table(rows, a.table_out)
    if a.cmd in ("curves", "all"):
        cmd_curves(rows, a.fig_dir, metrics, not a.linear_ate)
    if a.cmd in ("case", "all"):
        runs = [(r, None) for r in a.run]
        if not runs and a.auto:
            runs = auto_cases(rows, a.runs_dir, a.auto, a.prefer_seq)
        for rd, drop in runs:
            name = os.path.basename(os.path.normpath(rd))
            cmd_case(rd, os.path.join(a.fig_dir, f"case_{name}.png"))
            if drop is not None:
                print(f"       coverage drop vs L0 (mean over seeds): {drop:.3f}")


if __name__ == "__main__":
    main()
