"""PPT figures for A (baseline) from results/A/baseline_metrics.csv + baseline_runs.csv (read-only on B/C runs).

  01_baseline_bars_<seq>.png   per mode: online ATE (segmented), final ATE, RPE, coverage, segment count (mean, min-max)
  02_traj_<seq>.png            top view (x-y) of GT and each mode's final trajectory for one seed (SE3/Sim3 aligned)
  03_concurrency.png           per-frame latency and IMU_MONOCULAR coverage vs number of concurrent runs
Run with MPLCONFIGDIR redirected (source scripts/docs_env.sh). Usage:
  python plot_baseline.py [--metrics-dir results/A] [--out results/A/figures] [--seed 0] [--lang zh|en]
--lang en (default; scientific-figure convention) or zh, which uses the static Noto Serif CJK SC font from env_docs
(matplotlib cannot read the -VF fonts);
mode names and metric abbreviations (ATE, RPE, SE(3)) stay in English.
"""
import argparse, csv, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker
from matplotlib import font_manager

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
ZH_FONT = os.path.join(ROOT, "env_docs/share/fonts/noto-serif-cjk-sc/NotoSerifCJKsc-Regular.otf")
plt.rcParams.update({"axes.unicode_minus": False, "figure.dpi": 150, "savefig.bbox": "tight", "font.size": 11})

MODES = ["MONOCULAR", "IMU_MONOCULAR", "RGBD", "IMU_RGBD"]
LABEL = {"MONOCULAR": "RGB", "IMU_MONOCULAR": "RGB+IMU", "RGBD": "RGB-D", "IMU_RGBD": "RGB-D+IMU"}
COLOR = {"MONOCULAR": "#4C72B0", "IMU_MONOCULAR": "#DD8452", "RGBD": "#55A868", "IMU_RGBD": "#C44E52"}
TEXT = {
    "en": {
        "seq_easy": "easy (TartanAir V2 Office P004)", "seq_hard": "hard (TartanAir V2 Office P004)",
        "p_ate_on": "Online ATE RMSE (per-segment align) [m]", "p_ate_fin": "Final-trajectory ATE RMSE [m]",
        "p_rpe": "RPE trans. (Δ=1 s) RMSE [m]", "p_cov": "Valid-pose coverage",
        "p_seg": "# aligned segments (more = optimistic ATE)", "no_run": "no run", "no_result": "no result",
        "bars_title": "Fusion baselines · {seq} · bar = mean over seeds, whisker = min–max (runs {n})",
        "gt": "ground truth", "final": "final trajectory (aligned)", "frames": "{k}/{n} frames",
        "frames_ate": "{k}/{n} frames · ATE {ate:.2f} m", "no_final": "no final trajectory (save failed / empty map)",
        "aligned": "{mode} ({al} aligned)", "x": "x (north) [m]", "y": "y (east) [m]",
        "traj_title": "Top view · {seq} · seed {seed} · final trajectory = largest map, online-OK frames only",
        "mean": "{mode} mean", "p95": "{mode} p95", "budget": "10 Hz real-time budget (100 ms)",
        "conc_x": "concurrent ORB-SLAM3 runs", "lat_y": "per-frame tracking time [ms]", "cov_y": "valid-pose coverage",
        "conc_title": "Effect of concurrent runs (easy, container CPU quota = 4 cores; green = adopted limit 4)",
    },
    "zh": {
        "seq_easy": "easy（TartanAir V2 Office P004）", "seq_hard": "hard（TartanAir V2 Office P004）",
        "p_ate_on": "在线 ATE RMSE（分段对齐）/ m", "p_ate_fin": "最终轨迹 ATE RMSE / m",
        "p_rpe": "RPE 平移 RMSE（Δ=1 s）/ m", "p_cov": "有效位姿覆盖率",
        "p_seg": "对齐段数（越多 ATE 越乐观）", "no_run": "无运行", "no_result": "无结果",
        "bars_title": "融合基线 · {seq} · 柱 = 种子均值，误差线 = 最小–最大（运行数 {n}）",
        "gt": "真值", "final": "最终轨迹（对齐后）", "frames": "{k}/{n} 帧",
        "frames_ate": "{k}/{n} 帧 · ATE {ate:.2f} m", "no_final": "无最终轨迹（保存失败或地图为空）",
        "aligned": "{mode}（{al} 对齐）", "x": "x（北）/ m", "y": "y（东）/ m",
        "traj_title": "轨迹俯视图 · {seq} · 种子 {seed} · 最终轨迹 = 最大地图中在线 OK 的帧",
        "mean": "{mode} 均值", "p95": "{mode} p95", "budget": "10 Hz 实时预算（100 ms）",
        "conc_x": "同时运行的 ORB-SLAM3 个数", "lat_y": "每帧跟踪耗时 / ms", "cov_y": "有效位姿覆盖率",
        "conc_title": "并发对结果的影响（easy，容器 CPU 配额 4 核；绿线 = 采用的上限 4）",
    },
}
L = TEXT["en"]


def set_lang(lang):
    global L
    L = TEXT[lang]
    if lang == "zh":
        if not os.path.exists(ZH_FONT):
            raise SystemExit(f"Chinese font not found: {ZH_FONT} (use --lang en)")
        font_manager.fontManager.addfont(ZH_FONT)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=ZH_FONT).get_name()


def seq_name(seq):
    return L.get("seq_" + seq.split("_")[1], seq) if seq.count("_") >= 1 else seq


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def read_csv(p):
    return list(csv.DictReader(open(p))) if os.path.exists(p) else []


def pick(rows, seq, mode):
    """Prefer the source with more runs (B and C both run IMU_RGBD clean)."""
    c = [r for r in rows if r["sequence"] == seq and r["mode"] == mode]
    return max(c, key=lambda r: int(r["n_runs"])) if c else None


def bars(metrics, seq, out):
    panels = [("ate_online_rmse", L["p_ate_on"], True), ("ate_final_rmse", L["p_ate_fin"], True),
              ("rpe_trans_rmse", L["p_rpe"], True), ("coverage", L["p_cov"], False), ("n_segments", L["p_seg"], False)]
    fig, axs = plt.subplots(1, len(panels), figsize=(4.0 * len(panels), 4.2))
    for ax, (k, title, logy) in zip(axs, panels):
        for i, mode in enumerate(MODES):
            r = pick(metrics, seq, mode)
            m = fnum(r and r.get(f"{k}_mean"))
            if m is None:
                ax.text(i, 0.03, L["no_run"] if r is None else L["no_result"], ha="center", va="bottom", rotation=90,
                        transform=ax.get_xaxis_transform(), color="0.4", fontsize=9)
                continue
            lo, hi = fnum(r[f"{k}_min"]), fnum(r[f"{k}_max"])
            ax.bar(i, m, color=COLOR[mode], width=0.65)
            ax.errorbar(i, m, yerr=[[m - lo], [hi - m]], color="k", capsize=4, lw=1)
            ax.text(i, hi, f"{m:.0f}" if k == "n_segments" or m >= 100 else f"{m:.2f}", ha="center", va="bottom", fontsize=8)
        ax.set_xticks(range(len(MODES)), [LABEL[m] for m in MODES], rotation=25)
        ax.set_xlim(-0.6, len(MODES) - 0.4)
        ax.set_title(title, fontsize=10)
        if logy:
            ax.set_yscale("log")
            ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
            # label minor ticks only when the axis spans less than ~1.5 decades
            lo_, hi_ = ax.get_ylim()
            ax.yaxis.set_minor_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}")
                                         if hi_ / lo_ < 30 else matplotlib.ticker.NullFormatter())
        if k == "coverage":
            ax.set_ylim(0, 1.08)
        ax.grid(axis="y", alpha=0.3)
    n = {m: (pick(metrics, seq, m) or {}).get("n_runs", "0") for m in MODES}
    fig.suptitle(L["bars_title"].format(seq=seq_name(seq), n="/".join(n[m] for m in MODES)), fontsize=12)
    fig.savefig(out); plt.close(fig)


def load_tum(p):
    if not os.path.exists(p):
        return {}
    d = {}
    for l in open(p):
        v = l.split()
        if len(v) == 8:
            d[round(float(v[0]), 3)] = np.array(v[1:4], float)
    return d


def align(X, Y, scale):
    mx, my = X.mean(1, keepdims=True), Y.mean(1, keepdims=True)
    U, D, V = np.linalg.svd((Y - my) @ (X - mx).T / X.shape[1]); S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(V) < 0:
        S[2, 2] = -1
    s = np.trace(np.diag(D) @ S) / ((X - mx) ** 2).sum(0).mean() if scale else 1.0
    R = U @ S @ V
    return lambda P: s * R @ (P - mx) + my


def traj(runs, seq, seed, out):
    gt = load_tum(os.path.join(ROOT, "data/seqs", seq, "trajectory_gt.tum"))
    if not gt:
        return False
    G = np.array([gt[t] for t in sorted(gt)]).T
    fig, axs = plt.subplots(1, 4, figsize=(16, 4.6), sharex=True, sharey=True)
    for ax, mode in zip(axs, MODES):
        ax.plot(G[0], G[1], color="0.55", lw=1.2, label=L["gt"])
        c = [r for r in runs if r["sequence"] == seq and r["mode"] == mode and int(r["seed"]) == seed]
        c.sort(key=lambda r: r["source"])
        note = L["no_run"]
        if c:
            r = c[0]
            est = load_tum(os.path.join(ROOT, r["run_dir"], "trajectory_est.tum"))
            k = [t for t in sorted(est) if t in gt]
            if len(k) >= 3:
                X = np.array([est[t] for t in k]).T; Y = np.array([gt[t] for t in k]).T
                A = align(X, Y, mode == "MONOCULAR")(X)
                ax.plot(A[0], A[1], ".", ms=2.2, color=COLOR[mode], label=L["final"])
                ate = fnum(r["ate_final_rmse"])
                note = L["frames_ate"].format(k=len(k), n=len(gt), ate=ate) if ate is not None \
                    else L["frames"].format(k=len(k), n=len(gt))
            else:
                note = L["no_final"]
            note += f"\n{r['source']}: {os.path.basename(r['run_dir'])}"
        ax.set_title(L["aligned"].format(mode=LABEL[mode], al="Sim(3)" if mode == "MONOCULAR" else "SE(3)"), fontsize=11)
        ax.text(0.02, 0.02, note, transform=ax.transAxes, fontsize=7, va="bottom", color="0.25")
        ax.set_xlabel(L["x"]); ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=0.3)
    axs[0].set_ylabel(L["y"]); axs[0].legend(loc="upper right", fontsize=8)
    fig.suptitle(L["traj_title"].format(seq=seq_name(seq), seed=seed), fontsize=12)
    fig.savefig(out); plt.close(fig)
    return True


def concurrency(out):
    p = os.path.join(ROOT, "results/A/T1b_concurrency/summary.csv")
    rows = read_csv(p)
    if not rows:
        return False
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4))
    for mode in ("IMU_RGBD", "IMU_MONOCULAR"):
        g = sorted([r for r in rows if r["mode"] == mode], key=lambda r: int(r["level"]))
        x = [int(r["level"]) for r in g]
        a1.plot(x, [float(r["lat_mean_ms_mean"]) for r in g], "o-", color=COLOR[mode], label=L["mean"].format(mode=LABEL[mode]))
        a1.plot(x, [float(r["lat_p95_ms_mean"]) for r in g], "o--", color=COLOR[mode], alpha=0.6, label=L["p95"].format(mode=LABEL[mode]))
        a2.errorbar(x, [float(r["coverage_mean"]) for r in g],
                    yerr=[[float(r["coverage_mean"]) - float(r["coverage_min"]) for r in g],
                          [float(r["coverage_max"]) - float(r["coverage_mean"]) for r in g]],
                    fmt="o-", capsize=3, color=COLOR[mode], label=LABEL[mode])
    a1.axhline(100, color="k", lw=0.8, ls=":"); a1.text(1.05, 104, L["budget"], fontsize=8)
    for a in (a1, a2):
        a.set_xscale("log", base=2); a.set_xticks([1, 2, 4, 6, 16, 32], ["1", "2", "4", "6", "16", "32"])
        a.axvline(4, color="g", lw=0.8, ls="--"); a.set_xlabel(L["conc_x"]); a.grid(alpha=0.3)
    a1.set_ylabel(L["lat_y"]); a1.set_yscale("log"); a1.legend(fontsize=8)
    a2.set_ylabel(L["cov_y"]); a2.set_ylim(0, 1.05); a2.legend(fontsize=8)
    fig.suptitle(L["conc_title"], fontsize=12)
    fig.savefig(out); plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--metrics-dir", default="results/A")
    ap.add_argument("--out", default="results/A/figures")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--lang", choices=["zh", "en"], default="en")
    a = ap.parse_args()
    set_lang(a.lang)
    md, od = os.path.join(ROOT, a.metrics_dir), os.path.join(ROOT, a.out)
    os.makedirs(od, exist_ok=True)
    metrics, runs = read_csv(os.path.join(md, "baseline_metrics.csv")), read_csv(os.path.join(md, "baseline_runs.csv"))
    made = []
    for seq in sorted({r["sequence"] for r in metrics}):
        tag = seq.split("_")[1]
        bars(metrics, seq, os.path.join(od, f"01_baseline_bars_{tag}.png")); made.append(f"01_baseline_bars_{tag}.png")
        if traj(runs, seq, a.seed, os.path.join(od, f"02_traj_{tag}.png")):
            made.append(f"02_traj_{tag}.png")
    if concurrency(os.path.join(od, "03_concurrency.png")):
        made.append("03_concurrency.png")
    print("wrote", ", ".join(made), "->", od)


if __name__ == "__main__":
    main()
