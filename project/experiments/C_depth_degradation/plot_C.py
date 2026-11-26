#!/usr/bin/env python
"""Figures for C (depth degradation), from the per-run CSV written by scripts/aggregate.py.

Usage:
    python experiments/C_depth_degradation/plot_C.py --runs results/C/depth_ablation_runs.csv \
        --out results/C/figures [--sequence office_easy_p004] [--lang zh|en]

--lang en (default; scientific-figure convention) or zh: zh puts titles, axes and legends in Chinese (static Noto Serif CJK SC from env_docs; matplotlib
cannot read the variable CJK font); mode names (RGBD, IMU_RGBD, IMU_MONOCULAR) and ATE/RPE stay in English.

Figures (one PNG per sequence; every number comes from the runs CSV):
    01_invalid_ratio_<seq>.png   x = invalid depth ratio (P00/P20/P50/P80 + clean), three stacked panels:
                                 online ATE (m), coverage, and number of (map_id, frame_epoch) segments
                                 (solid = all segments, dashed = segments actually used for the ATE);
                                 RGBD vs IMU_RGBD, IMU_MONOCULAR clean as a horizontal reference ("RGB+IMU")
    02_noise_<seq>.png           x = relative depth noise r (0, 1, 2, 5, 10 %), same three panels
    03_outage_<seq>.png          x = outage duration (0.5/1/2 s): fraction of valid frames inside the outage
                                 window (computed from tracking.csv) and total lost events
    04_status_<seq>.png          outcome per (fault level, mode): runs ok / init failed / other, so that
                                 "normal / init failure / reset" for many-invalid or no depth is explicit

Point = mean over seeds, error bar = std over seeds (only seeds where the metric exists; the count n
is printed next to each point when it is below the number of runs). Runs that did not initialise have
no ATE and are therefore absent from the ATE panel but present in the coverage panel (coverage 0).
"""
import argparse
import csv
import os
import statistics
import textwrap

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.ticker import MaxNLocator  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
ZH_FONT = os.path.join(ROOT, "env_docs", "share", "fonts", "noto-serif-cjk-sc", "NotoSerifCJKsc-Regular.otf")

STR = {
    "zh": {
        "ref": "IMU_MONOCULAR（无扰动）",
        "ate": "在线 ATE RMSE（m）", "cov": "覆盖率（有效帧 / 总帧）",
        "rec": "中断后恢复时间（s）", "lost": "失跟事件（次）", "runs": "运行次数（种子）",
        "win_valid": "中断窗口内有效帧比例",
        "seg": "分段数（个）", "seg_all": "全部分段", "seg_aligned": "参与 ATE 的分段",
        "x_invalid": "加入的无效深度像素比例（%）", "x_noise": "相对深度噪声 r（σ = r·z，%）",
        "x_outage": "整帧深度中断时长（s）",
        "t_invalid": "C · 无效深度比例与 ATE、覆盖率（{seq}）", "t_noise": "C · 深度噪声与 ATE、覆盖率（{seq}）",
        "t_outage": "C · 深度中断（{seq}）", "t_status": "C · 各故障档位的运行结局（{seq}）",
        "ok": "正常", "init_failed": "初始化失败", "other": "其他失败", "clean": "无扰动",
        "foot_two": "点 = 种子均值，误差棒 = 种子间标准差；在线 ATE 只在按 (map_id, frame_epoch) 对齐的分段上计算，"
                    "分段越多、参与 ATE 的帧越少，ATE 会偏小，须与分段数和覆盖率一起看；"
                    "有效帧 = 状态 OK 且跟踪点数 ≥ 1；数据：{src}",
        "foot_outage": "上：深度中断窗口 [开始, 结束) 内有效帧（状态 OK 且跟踪点数 ≥ 1）的比例，"
                       "1.0 = 中断期间跟踪从未中断（窗口结束后算的恢复时间在本数据 easy 全部运行中为 0 s；"
                       "两线重合时 RGBD 画得更粗、点更大，避免被 IMU_RGBD 遮住）；"
                       "下：整次运行的失跟事件总数，不受窗口限制（easy RGBD 的事件都发生在窗口外，"
                       "hard 无扰动基线本身反复重置）；有效帧规则同 scripts/evaluate.py；数据：{src}",
        "foot_status": "数据：{src}",
    },
    "en": {
        "ref": "IMU_MONOCULAR (clean)",
        "ate": "Online ATE RMSE (m)", "cov": "Coverage (valid frames / all)",
        "rec": "Recovery time after outage (s)", "lost": "Lost events (count)", "runs": "Runs (seeds)",
        "win_valid": "Valid frames inside outage window (fraction)",
        "seg": "Segments (count)", "seg_all": "all segments", "seg_aligned": "used for ATE",
        "x_invalid": "Invalid depth pixels added (%)", "x_noise": "Relative depth noise r (σ = r·z, %)",
        "x_outage": "Whole-frame depth outage duration (s)",
        "t_invalid": "C · invalid depth ratio vs ATE and coverage ({seq})",
        "t_noise": "C · depth noise vs ATE and coverage ({seq})",
        "t_outage": "C · depth outage ({seq})", "t_status": "C · run outcome per fault level ({seq})",
        "ok": "ok", "init_failed": "init failed", "other": "other failure", "clean": "clean",
        "foot_two": "mean ± std over seeds; online ATE is computed only on segments aligned per "
                    "(map_id, frame_epoch), so more segments (fewer frames in the ATE) bias ATE low: "
                    "read ATE together with segments and coverage; "
                    "valid = state OK and ≥1 tracked point; source: {src}",
        "foot_outage": "top: fraction of frames inside the outage window [start, end) that are valid "
                       "(state OK, >=1 tracked point); 1.0 = tracking never broke during the outage "
                       "(recovery time measured after the window is 0 s in every easy run; "
                       "where the two curves coincide, RGBD is drawn thicker/larger so it is not hidden). "
                       "bottom: total lost events in the whole run, not window-restricted "
                       "(every easy RGBD event lies outside the window; hard baseline resets repeatedly). "
                       "valid-frame rule as in scripts/evaluate.py; source: {src}",
        "foot_status": "source: {src}",
    },
}
S = STR["en"]


def set_lang(lang):
    global S
    S = STR[lang]
    if lang == "zh":
        if not os.path.isfile(ZH_FONT):
            raise SystemExit(f"Chinese font not found: {ZH_FONT} (use --lang en)")
        font_manager.fontManager.addfont(ZH_FONT)
        matplotlib.rcParams["font.family"] = font_manager.FontProperties(fname=ZH_FONT).get_name()
        matplotlib.rcParams["axes.unicode_minus"] = False
    MODE_STYLE["IMU_MONOCULAR"]["label"] = S["ref"]

# Reference categorical palette, first three slots (validated: CVD/normal-vision pass on #fcfcfb;
# aqua < 3:1 contrast -> every line is direct-labelled and has its own marker).
MODE_STYLE = {
    "RGBD": {"color": "#2a78d6", "marker": "o", "label": "RGBD"},
    "IMU_RGBD": {"color": "#eb6834", "marker": "s", "label": "IMU_RGBD"},
    "IMU_MONOCULAR": {"color": "#1baf7a", "marker": "^", "label": "IMU_MONOCULAR (clean)"},
}
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT2 = "#52514e"
GRID = "#e4e3df"

INVALID_X = {"clean": 0.0, "P00": 0.0, "P20": 20.0, "P50": 50.0, "P80": 80.0}
NOISE_X = {"clean": 0.0, "N1": 1.0, "N2": 2.0, "N3": 5.0, "N4": 10.0}
OUTAGE_X = {"O05": 0.5, "O10": 1.0, "O20": 2.0}


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


MIN_TRACKED_POINTS = 1


def outage_window_valid_fraction(run, runs_dir):
    """Fraction of frames inside [outage_start_s, outage_end_s) that are valid.

    Same rule as scripts/evaluate.py: state OK (tracking.csv valid == 1) and quality_rgb >=
    MIN_TRACKED_POINTS. The denominator counts every tracking row whose timestamp falls in the
    window, so 1.0 means the depth outage never interrupted tracking. Returns None if the run has
    no outage window, no tracking.csv, or no frame in the window.
    """
    t0, t1 = fnum(run.get("outage_start_s")), fnum(run.get("outage_end_s"))
    p = os.path.join(runs_dir, run["run"], "tracking.csv")
    if t0 is None or t1 is None or not os.path.isfile(p):
        return None
    n = n_valid = 0
    with open(p) as f:
        for r in csv.DictReader(f):
            if None in r.values():  # truncated last line of a crashed run
                continue
            t = fnum(r.get("timestamp"))
            if t is None or not (t0 <= t < t1):
                continue
            n += 1
            pts = fnum(r.get("quality_rgb")) or 0.0
            if int(fnum(r.get("valid")) or 0) == 1 and pts >= MIN_TRACKED_POINTS:
                n_valid += 1
    return n_valid / n if n else None


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    for s in ["top", "right"]:
        ax.spines[s].set_visible(False)
    for s in ["left", "bottom"]:
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=TEXT2, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def series(rows, mode, fault_x, metric, faults):
    """x -> (mean, std, n_metric, n_runs) for one mode over the given faults (+ clean)."""
    pts = {}
    for r in rows:
        if r["mode"] != mode:
            continue
        key = "clean" if r["fault"] == "clean" else r["level"]
        if r["fault"] != "clean" and r["fault"] not in faults:
            continue
        if key not in fault_x:
            continue
        pts.setdefault(fault_x[key], []).append(fnum(r.get(metric)))
    out = {}
    for x, vals in sorted(pts.items()):
        v = [y for y in vals if y is not None]
        out[x] = (statistics.fmean(v) if v else None, statistics.stdev(v) if len(v) > 1 else 0.0, len(v), len(vals))
    return out


def plot_line(ax, s, mode, last_label=True, markersize=7, linewidth=2):
    st = MODE_STYLE[mode]
    xs = [x for x, p in s.items() if p[0] is not None]
    if not xs:
        return
    ys = [s[x][0] for x in xs]
    es = [s[x][1] for x in xs]
    ax.errorbar(xs, ys, yerr=es, color=st["color"], marker=st["marker"], markersize=markersize,
                linewidth=linewidth, capsize=3, markeredgecolor=SURFACE, markeredgewidth=1.5,
                label=st["label"], zorder=3)
    for x in xs:
        mean, _, n, nr = s[x]
        if n < nr:
            ax.annotate(f"n={n}/{nr}", (x, mean), textcoords="offset points", xytext=(6, -12),
                        fontsize=8, color=TEXT2)
    if last_label:
        ax._end_labels = getattr(ax, "_end_labels", []) + [(xs[-1], ys[-1], st["label"])]


def place_end_labels(ax, min_gap_px=22):
    """Direct labels at line ends, nudged apart vertically so they never overlap."""
    labs = getattr(ax, "_end_labels", [])
    if not labs:
        return
    fig = ax.figure
    fig.canvas.draw()
    pts = sorted(((ax.transData.transform((x, y))[1], x, y, t) for x, y, t in labs))
    placed = []
    for py, x, y, t in pts:
        target = py if not placed else max(py, placed[-1] + min_gap_px)
        placed.append(target)
        dy = (target - py) * 72.0 / fig.dpi
        ax.annotate(t, (x, y), textcoords="offset points", xytext=(8, dy), fontsize=9, color=TEXT, va="center")


def reference_line(ax, rows, metric, seq):
    vals = [fnum(r.get(metric)) for r in rows if r["mode"] == "IMU_MONOCULAR" and r["fault"] == "clean"]
    vals = [v for v in vals if v is not None]
    if not vals:
        return
    st = MODE_STYLE["IMU_MONOCULAR"]
    m = statistics.fmean(vals)
    ax.axhline(m, color=st["color"], linewidth=2, linestyle=(0, (4, 3)), zorder=2, label=st["label"])
    ax.annotate(f"{st['label']}: {m:.3g}", (0.0, m), xycoords=("axes fraction", "data"),
                textcoords="offset points", xytext=(4, 4), ha="left", fontsize=8, color=TEXT2)


def footer(fig, text):
    width = 70 if S is STR["zh"] else 120  # CJK glyphs are about twice as wide
    fig.text(0.02, 0.005, "\n".join(textwrap.wrap(text, width)), fontsize=7, color=TEXT2, va="bottom")


def seg_lines(ax, s, mode):
    """Total segments (solid) and segments actually used for the ATE (dashed), mean over seeds."""
    st = MODE_STYLE[mode]
    for metric, ls, lw, mk in (("n_segments", "-", 2, st["marker"]),
                               ("n_segments_aligned", (0, (4, 3)), 1.5, None)):
        pts = s[metric]
        xs = [x for x, p in pts.items() if p[0] is not None]
        if not xs:
            continue
        ax.plot(xs, [pts[x][0] for x in xs], color=st["color"], linestyle=ls, linewidth=lw,
                marker=mk, markersize=6, markeredgecolor=SURFACE, markeredgewidth=1.5,
                alpha=1.0 if ls == "-" else 0.75, zorder=3)


def two_panel(rows, seq, fault, fault_x, xlabel, title, out, src):
    fig = plt.figure(figsize=(7.5, 9.2), facecolor=SURFACE)
    gs = fig.add_gridspec(3, 1, height_ratios=[1.25, 1, 0.9], hspace=0.12)
    ax0 = fig.add_subplot(gs[0])
    axes = [ax0, fig.add_subplot(gs[1], sharex=ax0), fig.add_subplot(gs[2], sharex=ax0)]
    panels = [("ate_online_rmse", S["ate"]), ("coverage", S["cov"])]
    for ax, (metric, ylab) in zip(axes, panels):
        style_axes(ax)
        for mode in ["RGBD", "IMU_RGBD"]:
            plot_line(ax, series(rows, mode, fault_x, metric, {fault}), mode)
        reference_line(ax, rows, metric, seq)
        ax.set_ylabel(ylab, color=TEXT, fontsize=10)
    axes[1].set_ylim(0, 1.05)
    axes[1].set_yticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
    # third panel: number of (map_id, frame_epoch) segments, so a small ATE cannot be read alone
    style_axes(axes[2])
    for mode in ["RGBD", "IMU_RGBD"]:
        seg_lines(axes[2], {m: series(rows, mode, fault_x, m, {fault})
                            for m in ("n_segments", "n_segments_aligned")}, mode)
    axes[2].set_ylabel(S["seg"], color=TEXT, fontsize=10)
    axes[2].set_ylim(bottom=0)
    axes[2].yaxis.set_major_locator(MaxNLocator(nbins=5, integer=True, prune=None))
    for ax in axes:
        place_end_labels(ax)
    axes[2].set_xlabel(xlabel, color=TEXT, fontsize=10)
    axes[2].set_xticks(sorted(set(fault_x.values())))
    # legend above the plots, outside the data area, so it never covers points
    handles = [Line2D([], [], color=MODE_STYLE[m]["color"], marker=MODE_STYLE[m]["marker"],
                      markeredgecolor=SURFACE, linewidth=2, label=MODE_STYLE[m]["label"])
               for m in ("RGBD", "IMU_RGBD", "IMU_MONOCULAR")]
    handles += [Line2D([], [], color=TEXT2, linestyle="-", linewidth=2, label=S["seg_all"]),
                Line2D([], [], color=TEXT2, linestyle=(0, (4, 3)), linewidth=1.5, label=S["seg_aligned"])]
    axes[0].legend(handles=handles, frameon=False, fontsize=9, ncol=3,
                   loc="lower left", bbox_to_anchor=(0.0, 1.02))
    fig.suptitle(title, color=TEXT, fontsize=12, x=0.02, ha="left")
    footer(fig, S["foot_two"].format(src=src))
    fig.subplots_adjust(left=0.095, right=0.875, top=0.90, bottom=0.125, hspace=0.16)
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def outage_fig(rows, seq, out, src):
    fig, axes = plt.subplots(2, 1, figsize=(7.5, 7.6), sharex=True, facecolor=SURFACE)
    panels = [("outage_window_valid_frac", S["win_valid"]), ("lost_events", S["lost"])]
    for ax, (metric, ylab) in zip(axes, panels):
        style_axes(ax)
        for mode in ["RGBD", "IMU_RGBD"]:
            # In the fraction panel both curves sit exactly at 1.0 on easy; RGBD is drawn thicker and with
            # a larger marker so it stays visible under IMU_RGBD (otherwise the later curve hides it).
            sized = metric == "outage_window_valid_frac"
            plot_line(ax, series(rows, mode, OUTAGE_X, metric, {"outage"}), mode,
                      markersize=10 if sized and mode == "RGBD" else 6.5 if sized else 7,
                      linewidth=3 if sized and mode == "RGBD" else 2)
        ax.set_ylabel(ylab, color=TEXT, fontsize=10)
        place_end_labels(ax)
    # fraction scale anchored at 0; headroom shows the upper error bars (hard seeds vary 0-1, so mean+std > 1)
    axes[0].set_ylim(0, 1.15)
    axes[0].set_yticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
    axes[1].set_xlabel(S["x_outage"], color=TEXT, fontsize=10)
    axes[1].set_xticks(list(OUTAGE_X.values()))
    axes[0].legend(frameon=False, fontsize=9, ncol=2, loc="lower left", bbox_to_anchor=(0.0, 1.02))
    fig.suptitle(S["t_outage"].format(seq=seq), color=TEXT, fontsize=12, x=0.02, ha="left")
    footer(fig, S["foot_outage"].format(src=src))
    fig.subplots_adjust(left=0.115, right=0.875, top=0.88, bottom=0.16, hspace=0.30)
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def status_fig(rows, seq, out, src):
    order = ["clean", "P00", "P20", "P50", "P80", "N1", "N2", "N3", "N4", "O05", "O10", "O20", "ALL"]
    cats = [("ok", "#2a78d6", S["ok"]), ("init_failed", "#eb6834", S["init_failed"]), ("other", "#8a8984", S["other"])]
    modes = ["RGBD", "IMU_RGBD"]
    labels, counts = [], []
    for mode in modes:
        for lv in order:
            rs = [r for r in rows if r["mode"] == mode and (r["level"] or "clean") == lv]
            if not rs:
                continue
            c = {"ok": 0, "init_failed": 0, "other": 0}
            for r in rs:
                c[r["status"] if r["status"] in ("ok", "init_failed") else "other"] += 1
            labels.append(f"{MODE_STYLE[mode]['label']} · {S['clean'] if lv == 'clean' else lv}")
            counts.append(c)
    if not labels:
        return
    fig, ax = plt.subplots(figsize=(7.5, 0.32 * len(labels) + 1.6), facecolor=SURFACE)
    style_axes(ax)
    ax.grid(False, axis="y")
    y = list(range(len(labels)))[::-1]
    left = [0] * len(labels)
    for key, color, lab in cats:
        w = [c[key] for c in counts]
        ax.barh(y, w, left=left, color=color, height=0.7, edgecolor=SURFACE, linewidth=2, label=lab)
        left = [a + b for a, b in zip(left, w)]
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=8)
    ax.set_xlabel(S["runs"], color=TEXT, fontsize=10)
    ax.legend(frameon=False, fontsize=9, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0))
    fig.suptitle(S["t_status"].format(seq=seq), color=TEXT, fontsize=12, x=0.02, ha="left")
    footer(fig, S["foot_status"].format(src=src))
    fig.tight_layout(rect=(0, 0.02, 1, 0.95))
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--runs-dir", default=os.path.join(ROOT, "results", "C", "runs"),
                    help="directory of run folders with tracking.csv, used for the outage-window valid fraction")
    ap.add_argument("--sequence", action="append", default=None)
    ap.add_argument("--lang", choices=["zh", "en"], default="en")
    a = ap.parse_args()
    set_lang(a.lang)
    rows = load(a.runs)
    os.makedirs(a.out, exist_ok=True)
    src = os.path.basename(a.runs)
    for r in rows:
        if r.get("fault") == "outage":
            r["outage_window_valid_frac"] = outage_window_valid_fraction(r, a.runs_dir)
    seqs = a.sequence or sorted({r["base_sequence"] for r in rows})
    for seq in seqs:
        rs = [r for r in rows if r["base_sequence"] == seq]
        two_panel(rs, seq, "invalid_pixels", INVALID_X, S["x_invalid"], S["t_invalid"].format(seq=seq),
                  os.path.join(a.out, f"01_invalid_ratio_{seq}.png"), src)
        two_panel(rs, seq, "depth_noise", NOISE_X, S["x_noise"], S["t_noise"].format(seq=seq),
                  os.path.join(a.out, f"02_noise_{seq}.png"), src)
        outage_fig(rs, seq, os.path.join(a.out, f"03_outage_{seq}.png"), src)
        status_fig(rs, seq, os.path.join(a.out, f"04_status_{seq}.png"), src)
        print("figures for", seq, "->", a.out)


if __name__ == "__main__":
    main()
