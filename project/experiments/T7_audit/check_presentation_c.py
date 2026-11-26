#!/usr/bin/env python
"""T10 item 3 (C): compare numbers in presentation/C with results/C CSVs / metrics.json.

Read-only. Two parts:
  1. Tables 5.1 (easy) and 5.2 (hard) of presentation/C/实现说明.md vs results/C/depth_ablation.csv.
  2. Explicit numeric claims in 实现说明.md sections 6.x and in presentation/C/slides.md,
     recomputed from results/C/depth_ablation.csv / depth_ablation_runs.csv /
     results/C/runs/*/metrics.json.
MISMATCH lines are collected at the end. Exit status is always 0; the caller reads the text.
"""
import csv
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DOC = os.path.join(ROOT, "presentation/C/实现说明.md")
SLIDES = os.path.join(ROOT, "presentation/C/slides.md")


def num(v):
    if v in (None, ""):
        return None
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x


def starts(text, prefix):
    for ln in text.splitlines():
        if ln.startswith(prefix):
            return ln
    return None


class Checker:
    def __init__(self, groups, runs):
        self.bad = []
        self.n = 0
        self.g = groups
        self.runs = runs

    def ok(self, label, cond, got, want):
        self.n += 1
        if not cond:
            self.bad.append("%s: got=%s expected=%s" % (label, got, want))

    def close(self, label, got, want, tol):
        if got is None or want is None:
            self.bad.append("%s: parsed=%r expected=%r" % (label, got, want))
            self.n += 1
            return
        self.n += 1
        if abs(got - want) > tol:
            self.bad.append("%s: doc=%s csv=%s" % (label, got, want))

    def row(self, seq, fault, level, mode):
        return self.g.get((seq, fault, level, mode))


FAULT_MAP = {
    "无扰动": ("clean", ""),
    "invalid_pixels·P20": ("invalid_pixels", "P20"),
    "invalid_pixels·P50": ("invalid_pixels", "P50"),
    "invalid_pixels·P80": ("invalid_pixels", "P80"),
    "depth_noise·N2": ("depth_noise", "N2"),
    "depth_noise·N4": ("depth_noise", "N4"),
    "outage·O05": ("outage", "O05"),
    "outage·O10": ("outage", "O10"),
    "outage·O20": ("outage", "O20"),
    "depth_off": ("depth_off", "ALL"),
}


def parse_tables(chk, lines, start_marker, end_marker, seq):
    i = lines.index(start_marker)
    j = lines.index(end_marker)
    seen = False
    for ln in lines[i:j]:
        if not ln.startswith("|"):
            continue
        cells = [c.strip().replace("*", "") for c in ln.strip().strip("|").split("|")]
        if "故障" in cells[0]:
            seen = True
            continue
        if set(cells[0]) <= {"-"} or not seen:
            continue
        label, mode = cells[0], cells[1]
        fault, level = FAULT_MAP[label]
        r = chk.row(seq, fault, level, mode)
        if r is None:
            chk.bad.append("table: no CSV row for %s %s %s %s" % (seq, fault, level, mode))
            continue
        tag = "%s %s %s" % (seq.split("_")[1], label, mode)

        succ = cells[2]
        want_succ = "%d/%d" % (int(r["n_ok"]), int(r["n_runs"]))
        crash_note = "崩溃 1" if int(r["n_crashed"]) else None
        chk.ok(tag + " ok/runs", succ.startswith(want_succ), succ, want_succ)
        chk.ok(tag + " crash note", (crash_note is None and "崩溃" not in succ) or
               (crash_note == "崩溃 1" and "崩溃 1" in succ), succ, crash_note)

        # ATE mean±std
        cell = cells[3]
        if cell == "—":
            chk.ok(tag + " ate", num(r["ate_online_rmse_mean"]) is None, cell, "None")
        else:
            m, s = [x.strip() for x in cell.split("±")]
            chk.close(tag + " ate mean", float(m), num(r["ate_online_rmse_mean"]), 0.00051)
            chk.close(tag + " ate std", float(s), num(r["ate_online_rmse_std"]), 0.00051)

        def dash_or_value(col, csvcol, dec, tol, none_ok=None):
            c = cells[col]
            exp = num(r[csvcol])
            if c in ("—", "-"):
                chk.ok(tag + " " + csvcol, exp is None, c, "None")
            else:
                chk.close(tag + " " + csvcol, float(c), exp, tol)

        dash_or_value(4, "n_segments_mean", 1, 0.051)
        dash_or_value(5, "aligned_coverage_mean", 3, 0.00051)
        dash_or_value(6, "coverage_mean", 3, 0.00051)
        dash_or_value(7, "lost_events_mean", 1, 0.051)
        dash_or_value(8, "n_unrecovered_mean", 1, 0.051)

        fin = cells[9]
        exp = num(r["ate_final_rmse_mean"])
        if fin.startswith("—"):
            chk.ok(tag + " final", exp is None, fin, "None")
        else:
            m = fin.split("（")[0].split("(")[0].strip()
            chk.close(tag + " final mean", float(m), exp, 0.00051)
            if "n=" in fin:
                n = int(fin.split("n=")[1].split(")")[0].split("）")[0])
                chk.ok(tag + " final n", float(n) == float(r["ate_final_rmse_n"]),
                       n, r["ate_final_rmse_n"])


def main():
    groups = {}
    for r in csv.DictReader(open(os.path.join(ROOT, "results/C/depth_ablation.csv"))):
        groups[(r["base_sequence"], r["fault"], r["level"], r["mode"])] = r
    runs = list(csv.DictReader(open(os.path.join(ROOT, "results/C/depth_ablation_runs.csv"))))
    chk = Checker(groups, runs)
    lines = open(DOC).read().splitlines()
    slides = open(SLIDES).read()

    # ---- tables 5.1 / 5.2 ----
    parse_tables(chk, lines, "### 5.1 easy（`office_easy_p004`，主序列）",
                 "### 5.2 hard（`office_hard_p004`，副序列）", "office_easy_p004")
    parse_tables(chk, lines, "### 5.2 hard（`office_hard_p004`，副序列）", "## 6. 主要发现",
                 "office_hard_p004")

    # ---- status totals ----
    ok = sum(r["status"] == "ok" for r in runs)
    init = sum(r["status"] == "init_failed" for r in runs)
    fail = sum(r["status"] not in ("ok", "init_failed") for r in runs)
    chk.ok("status totals 162", len(runs) == 162, len(runs), 162)
    chk.ok("status totals ok", ok == 148, ok, 148)
    chk.ok("status totals init", init == 12, init, 12)
    chk.ok("status totals run_failed", fail == 2, fail, 2)

    # ---- 6.1 ----
    r = chk.row("office_easy_p004", "clean", "", "IMU_RGBD")
    rm = chk.row("office_easy_p004", "clean", "", "IMU_MONOCULAR")
    rr = chk.row("office_easy_p004", "clean", "", "RGBD")
    chk.close("6.1 imu_rgbd init 0.2", 0.2, num(r["init_delay_s_mean"]), 0.051)
    chk.close("6.1 imu_mono init 1.9", 1.9, num(rm["init_delay_s_mean"]), 0.051)
    chk.close("6.1 imu_mono valid 1115.7", 1115.7, num(rm["n_valid_mean"]), 0.051)
    chk.ok("6.1 imu_rgbd valid 1216/1218",
           abs(num(r["n_valid_mean"]) - 1216) < 0.05 and abs(num(r["n_frames_mean"]) - 1218) < 0.05,
           (r["n_valid_mean"], r["n_frames_mean"]), "1216/1218")
    chk.close("6.1 rgbd/imu at e ratio 2.7", 2.7,
              num(rr["ate_online_rmse_mean"]) / num(r["ate_online_rmse_mean"]), 0.051)
    chk.close("6.1 frames saved approx 100", 100.0,
              num(r["n_valid_mean"]) - num(rm["n_valid_mean"]), 0.6)
    chk.close("6.1 8% frames", 8.0,
              100.0 * (num(r["n_valid_mean"]) - num(rm["n_valid_mean"])) / num(r["n_frames_mean"]), 0.5)
    chk.close("6.1 segs quarter", 4.07, num(rm["n_segments_mean"]) / num(r["n_segments_mean"]), 0.06)

    # ---- 6.2 ----
    p20 = chk.row("office_easy_p004", "invalid_pixels", "P20", "RGBD")
    p50 = chk.row("office_easy_p004", "invalid_pixels", "P50", "RGBD")
    p80 = chk.row("office_easy_p004", "invalid_pixels", "P80", "RGBD")
    chk.close("6.2 p20 at e 0.103", 0.103, num(p20["ate_online_rmse_mean"]), 0.00051)
    chk.close("6.2 p50 at e 0.094", 0.094, num(p50["ate_online_rmse_mean"]), 0.00051)
    chk.close("6.2 p80 at e 0.026", 0.026, num(p80["ate_online_rmse_mean"]), 0.00051)
    chk.close("6.2 p80 segs 98.7", 98.7, num(p80["n_segments_mean"]), 0.051)
    chk.close("6.2 p80 aligned 0.850", 0.850, num(p80["aligned_coverage_mean"]), 0.00051)
    chk.close("6.2 p80 final 0.083", 0.083, num(p80["ate_final_rmse_mean"]), 0.00051)
    chk.ok("6.2 p80 final n=2", int(p80["ate_final_rmse_n"]) == 2, p80["ate_final_rmse_n"], 2)
    # IMU RGBD P80 lost/segs/cov
    ip80 = chk.row("office_easy_p004", "invalid_pixels", "P80", "IMU_RGBD")
    chk.close("6.2 imu p80 lost 7", 7.0, num(ip80["lost_events_mean"]), 0.051)
    chk.close("6.2 imu p80 segs 23", 23.0, num(ip80["n_segments_mean"]), 0.051)
    chk.close("6.2 imu p80 cov 0.987", 0.987, num(ip80["coverage_mean"]), 0.00051)

    # ---- 6.3 ----
    n1 = chk.row("office_easy_p004", "depth_noise", "N1", "RGBD")
    n2 = chk.row("office_easy_p004", "depth_noise", "N2", "RGBD")
    n3 = chk.row("office_easy_p004", "depth_noise", "N3", "RGBD")
    n4 = chk.row("office_easy_p004", "depth_noise", "N4", "RGBD")
    for lab, row, ate, segs, ali in [
            ("n1", n1, 0.095, 13.3, 0.981), ("n2", n2, 0.088, 17.0, 0.970),
            ("n3", n3, 0.067, 31.3, 0.933), ("n4", n4, 0.045, 86.0, 0.822)]:
        chk.close("6.3 rgbd %s ate %s" % (lab, ate), ate, num(row["ate_online_rmse_mean"]), 0.00051)
        chk.close("6.3 rgbd %s segs %s" % (lab, segs), segs, num(row["n_segments_mean"]), 0.051)
        chk.close("6.3 rgbd %s aligned %s" % (lab, ali), ali, num(row["aligned_coverage_mean"]), 0.00051)
    chk.close("6.3 rgbd cov 0.996", 0.996, num(n1["coverage_mean"]), 0.00051)
    chk.close("6.3 rgbd cov 0.921", 0.921, num(n4["coverage_mean"]), 0.00051)
    chk.close("6.3 rgbd lost 0.3", 0.3, num(n1["lost_events_mean"]), 0.051)
    chk.close("6.3 rgbd lost 5.0", 5.0, num(n4["lost_events_mean"]), 0.051)
    chk.close("6.3 rgbd final 0.166", 0.166, num(n4["ate_final_rmse_mean"]), 0.00051)
    chk.close("6.3 rgbd final/online 3.7", 3.7,
              num(n4["ate_final_rmse_mean"]) / num(n4["ate_online_rmse_mean"]), 0.051)
    in1 = chk.row("office_easy_p004", "depth_noise", "N1", "IMU_RGBD")
    in2 = chk.row("office_easy_p004", "depth_noise", "N2", "IMU_RGBD")
    in3 = chk.row("office_easy_p004", "depth_noise", "N3", "IMU_RGBD")
    in4 = chk.row("office_easy_p004", "depth_noise", "N4", "IMU_RGBD")
    for lab, row, ate in [("n1", in1, 0.030), ("n2", in2, 0.032), ("n3", in3, 0.034), ("n4", in4, 0.058)]:
        chk.close("6.3 imu %s ate %s" % (lab, ate), ate, num(row["ate_online_rmse_mean"]), 0.00051)
    chk.close("6.3 imu n4 +62%", 62.0,
              100.0 * (num(in4["ate_online_rmse_mean"]) / num(r["ate_online_rmse_mean"]) - 1.0), 0.6)
    chk.close("6.3 imu lost 2.3", 2.3, num(in4["lost_events_mean"]), 0.051)
    chk.close("6.3 imu segs 25", 25.0, num(in4["n_segments_mean"]), 0.051)
    # doc says coverage stays ~0.995 (N4 0.9945); rounded display 0.995
    cov_min = min(num(x["coverage_mean"]) for x in (in1, in2, in3, in4))
    chk.ok("6.3 imu coverage ~0.995 (N4)", abs(cov_min - 0.995) <= 0.0006, cov_min, "~0.995")
    chk.ok("6.3 doc states ~0.995 / 0.9945", "≈0.995（N4 为 0.9945）" in "\n".join(lines),
           "≈0.995（N4 为 0.9945）", "text")

    # ---- 6.4 ----
    o5 = chk.row("office_easy_p004", "outage", "O05", "RGBD")
    o10 = chk.row("office_easy_p004", "outage", "O10", "RGBD")
    o20 = chk.row("office_easy_p004", "outage", "O20", "RGBD")
    chk.close("6.4 o05 ate 0.187", 0.187, num(o5["ate_online_rmse_mean"]), 0.00051)
    chk.close("6.4 o05 std 0.150", 0.150, num(o5["ate_online_rmse_std"]), 0.00051)
    chk.close("6.4 o10 ate 0.066", 0.066, num(o10["ate_online_rmse_mean"]), 0.00051)
    chk.close("6.4 o20 ate 0.115", 0.115, num(o20["ate_online_rmse_mean"]), 0.00051)
    s1 = [x for x in runs if x["run"] == "office_easy_p004__outage_O05_s1__RGBD"][0]
    chk.close("6.4 o05 s1 0.358", 0.358, num(s1["ate_online_rmse"]), 0.00051)
    for lab, row in [("o05", o5), ("o10", o10), ("o20", o20)]:
        chk.ok("6.4 %s all recovered 3/3" % lab, int(row["outage_n_recovered"]) == 3,
               row["outage_n_recovered"], 3)
        chk.close("6.4 %s recovery 0.0" % lab, 0.0, num(row["outage_recovery_time_s_mean"]), 1e-9)
    easy_out = [x for x in runs if x["base_sequence"] == "office_easy_p004" and x["fault"] == "outage"]
    chk.ok("6.4 easy recovery all 0.0", all(num(x["outage_recovery_time_s"]) == 0.0 for x in easy_out),
           sorted({x["outage_recovery_time_s"] for x in easy_out}), "0.0")
    easy_rgbd_lost = [int(x["lost_events"]) for x in easy_out if x["mode"] == "RGBD"]
    # corrected text: group mean 1-2 per run, single-run range 0-4
    chk.ok("6.4/slides rgbd lost single-run 0-4",
           min(easy_rgbd_lost) == 0 and max(easy_rgbd_lost) == 4, easy_rgbd_lost, "0..4")
    grp = [num(chk.row("office_easy_p004", "outage", l, "RGBD")["lost_events_mean"])
           for l in ("O05", "O10", "O20")]
    chk.ok("6.4/slides rgbd lost group mean 1-2",
           min(grp) >= 1.0 and max(grp) <= 2.0, grp, "1..2")
    chk.ok("6.4 corrected wording present",
           "单次 0–4 次" in "\n".join(lines) and "单次 0–4 次" in slides
           and "组均 1–2 次" in "\n".join(lines) and "组均 1–2 次" in slides,
           "text", "group mean 1-2, single run 0-4")
    hard_clean = [x for x in runs if x["base_sequence"] == "office_hard_p004" and x["fault"] == "clean"]
    for m, base in (("RGBD", 30), ("IMU_RGBD", 150)):
        vals = [int(x["lost_events"]) for x in hard_clean if x["mode"] == m]
        chk.ok("6.4 hard clean %s lost ~%d" % (m, base), all(abs(v - base) <= 3 for v in vals), vals, base)
    unrec = [int(x["n_unrecovered"]) for x in hard_clean if x["mode"] in ("RGBD", "IMU_RGBD")]
    chk.ok("6.4 hard clean unrec 1-5", min(unrec) >= 1 and max(unrec) <= 5, unrec, "1..5")
    hard_out = [x for x in runs if x["base_sequence"] == "office_hard_p004" and x["fault"] == "outage"]
    hard_rec = sorted(num(x["outage_recovery_time_s"]) for x in hard_out)
    chk.ok("6.4/slides hard outage recovery single-run 0-4.8",
           min(hard_rec) == 0.0 and abs(max(hard_rec) - 4.8) < 1e-9, hard_rec, "0..4.8")
    hgrp = [num(chk.row("office_hard_p004", "outage", l, m)["outage_recovery_time_s_mean"])
            for m in ("RGBD", "IMU_RGBD") for l in ("O05", "O10", "O20")]
    chk.ok("6.4/slides hard outage recovery group mean 0.07-1.7",
           abs(min(hgrp) - 0.0666667) <= 0.001 and abs(max(hgrp) - 1.7) <= 1e-6,
           hgrp, "0.067..1.7")
    chk.ok("6.4 corrected hard recovery wording",
           "单次 0–4.8 s" in "\n".join(lines) and "组均 0.07–1.7 s" in "\n".join(lines),
           "text", "single run 0-4.8 s, group mean 0.07-1.7 s")
    # "hard 基线每次运行本来就有 146–151 次失跟": if "baseline" = no-fault clean runs,
    # IMU_RGBD is 148–152 and RGBD 29–32; 146–151 is the range of the hard outage IMU_RGBD runs.
    cl = {m: [int(x["lost_events"]) for x in hard_clean if x["mode"] == m] for m in ("RGBD", "IMU_RGBD")}
    chk.ok("6.4 'hard baseline lost 146-151' as no-fault clean IMU_RGBD",
           min(cl["IMU_RGBD"]) == 146 and max(cl["IMU_RGBD"]) == 151, cl, "146..151")

    # ---- 6.5 ----
    doff = [x for x in runs if x["fault"] == "depth_off"]
    chk.ok("6.5 depth_off 12 runs all init", len(doff) == 12 and all(x["status"] == "init_failed" for x in doff),
           (len(doff), {x["status"] for x in doff}), "12 init_failed")
    chk.ok("6.5 depth_off coverage 0",
           all(num(x["coverage"]) == 0.0 for x in doff), {x["coverage"] for x in doff}, 0.0)
    m = json.load(open(os.path.join(
        ROOT, "results/C/runs/office_easy_p004__depth_off_ALL_s0__RGBD/metrics.json")))
    chk.ok("6.5 n_ok_but_empty 609", m.get("n_ok_but_empty") == 609, m.get("n_ok_but_empty"), 609)
    d0 = [x for x in runs if x["run"] == "office_easy_p004__depth_off_ALL_s0__RGBD"][0]
    chk.close("6.5 wall 131.7", 131.7, num(d0["wall_s"]), 0.051)
    # state counts from tracking.csv
    st = {}
    for row in csv.DictReader(open(os.path.join(
            ROOT, "results/C/runs/office_easy_p004__depth_off_ALL_s0__RGBD/tracking.csv"))):
        st[row["state"]] = st.get(row["state"], 0) + 1
    chk.ok("6.5 tracking 609 state=2 / 609 state=4",
           st.get("2") == 609 and st.get("4") == 609, st, {"2": 609, "4": 609})

    # ---- 6.6 ----
    crashes = [x for x in runs if x["crashed"].lower() == "true"]
    chk.ok("6.6 two crashes", len(crashes) == 2, len(crashes), 2)
    ct = {x["run"]: num(x["crash_t_s"]) for x in crashes}
    chk.close("6.6 p20 crash t", 114.2, ct.get("office_easy_p004__invalid_pixels_P20_s2__RGBD"), 0.051)
    chk.close("6.6 p80 crash t", 111.7, ct.get("office_easy_p004__invalid_pixels_P80_s2__RGBD"), 0.051)
    chk.ok("6.6 exit 134", all(x["exit_code"] == "134" for x in crashes),
           {x["exit_code"] for x in crashes}, "134")
    chk.ok("6.6 crashed ATE n=3", int(chk.row("office_easy_p004", "invalid_pixels", "P20", "RGBD")
                                      ["ate_online_rmse_n"]) == 3, "n", 3)

    # ---- 6.7 ----
    easy = [x for x in runs if x["base_sequence"] == "office_easy_p004"]

    def loop_frac(mode):
        rs = [x for x in easy if x["mode"] == mode and x["status"] == "ok"]
        lo = [x for x in rs if num(x["n_loops"]) and num(x["n_loops"]) >= 1]
        return len(lo), len(rs)

    chk.ok("6.7 rgbd loops 23/34", loop_frac("RGBD") == (23, 34), loop_frac("RGBD"), (23, 34))
    chk.ok("6.7 imu_rgbd loops 10/36", loop_frac("IMU_RGBD") == (10, 36), loop_frac("IMU_RGBD"), (10, 36))
    chk.ok("6.7 imu_mono loops 1/3", loop_frac("IMU_MONOCULAR") == (1, 3), loop_frac("IMU_MONOCULAR"), (1, 3))
    hard_loops = [x for x in runs if x["base_sequence"] == "office_hard_p004" and num(x["n_loops"])]
    chk.ok("6.7 hard loops 0", len(hard_loops) == 0, len(hard_loops), 0)
    imu_loop = [x for x in easy if x["mode"].startswith("IMU") and num(x["n_loops"])]
    inflated = [x for x in imu_loop if num(x["ate_final_rmse"]) > 1.0]
    chk.ok("6.7 imu loop runs 11, inflated 9",
           len(imu_loop) == 11 and len(inflated) == 9, (len(imu_loop), len(inflated)), (11, 9))
    fin = sorted(num(x["ate_final_rmse"]) for x in inflated)
    chk.close("6.7 inflated final min 6.2", 6.2, fin[0], 0.051)
    chk.close("6.7 inflated final max 8.0", 8.0, fin[-1], 0.051)
    online = [num(x["ate_online_rmse"]) for x in imu_loop]
    chk.close("6.7 loop online min 0.024", 0.024, min(online), 0.00051)
    chk.close("6.7 loop online max 0.046", 0.046, max(online), 0.00051)
    rgbd_loop = [x for x in easy if x["mode"] == "RGBD" and num(x["n_loops"])]
    rf = sorted(num(x["ate_final_rmse"]) for x in rgbd_loop if num(x["ate_final_rmse"]) is not None)
    chk.close("6.7 rgbd loop final min 0.07", 0.07, rf[0], 0.0051)
    chk.close("6.7 rgbd loop final max 0.48", 0.48, rf[-1], 0.0051)
    # doc text says "RGBD 23/37 次成功运行"; successful (status ok) runs are 34.
    chk.ok("6.7 doc says RGBD 23/37 but ok runs = 34",
           "RGBD 23/34" in "\n".join(lines), "23/37", "23/34")
    # slides 7c: "10/36 IMU_RGBD loop runs, final 6.24-8.04" -> only 8 of 10 inflated
    imu_rgbd_loop = [x for x in easy if x["mode"] == "IMU_RGBD" and num(x["n_loops"])]
    imu_rgbd_infl = [x for x in imu_rgbd_loop if num(x["ate_final_rmse"]) > 1.0]
    chk.ok("slides 7c imu_rgbd loops 10, inflated 8",
           len(imu_rgbd_loop) == 10 and len(imu_rgbd_infl) == 8,
           (len(imu_rgbd_loop), len(imu_rgbd_infl)), (10, 8))
    chk.ok("slides 7c corrected 9/11 wording",
           "11 次回环运行" in slides and "其中 9 次的 final" in slides
           and "另 2 次为 0.040、0.053" in slides,
           "text", "11 loop runs, 9 inflated, other 2 = 0.040/0.053")
    chk.ok("6.7 corrected 11 loop runs / 9 inflated",
           "共有 11 次回环运行" in "\n".join(lines) and "其中 9 次的 final" in "\n".join(lines)
           and "IMU_RGBD 内是 8/10" in "\n".join(lines),
           "text", "11 loop runs, 9 inflated, IMU_RGBD 8/10")
    # enumerated 9 runs and their finals
    enum = {
        "office_easy_p004__clean_s0__IMU_MONOCULAR": 6.24,
        "office_easy_p004__invalid_pixels_P00_s0__IMU_RGBD": 7.50,
        "office_easy_p004__invalid_pixels_P00_s1__IMU_RGBD": 7.96,
        "office_easy_p004__invalid_pixels_P00_s2__IMU_RGBD": 7.90,
        "office_easy_p004__depth_noise_N1_s0__IMU_RGBD": 7.76,
        "office_easy_p004__depth_noise_N1_s2__IMU_RGBD": 8.02,
        "office_easy_p004__depth_noise_N2_s0__IMU_RGBD": 7.74,
        "office_easy_p004__outage_O05_s1__IMU_RGBD": 8.04,
        "office_easy_p004__outage_O10_s2__IMU_RGBD": 7.36,
    }
    byrun = {x["run"]: x for x in runs}
    for k, v in enum.items():
        chk.close("6.7 enumerated " + k.split("__", 1)[1], v, num(byrun[k]["ate_final_rmse"]), 0.0051)

    # ---- 6.9 ----
    hc = chk.row("office_hard_p004", "clean", "", "IMU_RGBD")
    hp80 = chk.row("office_hard_p004", "invalid_pixels", "P80", "IMU_RGBD")
    chk.close("6.9 aligned segs 24.7", 24.7, num(hp80["n_segments_aligned_mean"]), 0.051)
    chk.close("6.9 aligned segs 51.7", 51.7, num(hc["n_segments_aligned_mean"]), 0.051)
    chk.close("6.9 segs +21%", 21.0,
              100.0 * (num(hp80["n_segments_mean"]) / num(hc["n_segments_mean"]) - 1.0), 0.6)
    chk.close("6.9 aligned 0.129", 0.129, num(hp80["aligned_coverage_mean"]), 0.00051)
    chk.close("6.9 aligned 0.285", 0.285, num(hc["aligned_coverage_mean"]), 0.00051)
    chk.close("6.9 ate 0.0203", 0.0203, num(hp80["ate_online_rmse_mean"]), 0.000051)
    chk.close("6.9 ate ratio 0.18", 0.18,
              num(hp80["ate_online_rmse_mean"]) / num(hc["ate_online_rmse_mean"]), 0.0051)
    chk.close("6.9 cov 0.542", 0.542, num(hp80["coverage_mean"]), 0.00051)
    chk.close("6.9 unrec 8.3", 8.3, num(hp80["n_unrecovered_mean"]), 0.051)
    for lab, lv, segs, ate in [("p50", "P50", 157.7, 0.058), ("p20", "P20", 154.7, 0.061),
                               ("n2", "N2", 154.3, 0.067)]:
        fault = "invalid_pixels" if lv.startswith("P") else "depth_noise"
        row = chk.row("office_hard_p004", fault, lv, "IMU_RGBD")
        chk.close("6.9 %s segs %s" % (lab, segs), segs, num(row["n_segments_mean"]), 0.051)
        chk.close("6.9 %s ate %s" % (lab, ate), ate, num(row["ate_online_rmse_mean"]), 0.00051)
    ho10 = chk.row("office_hard_p004", "outage", "O10", "RGBD")
    ho20 = chk.row("office_hard_p004", "outage", "O20", "RGBD")
    hc_rgbd = chk.row("office_hard_p004", "clean", "", "RGBD")
    chk.close("6.9 o10 rgbd ate 0.151", 0.151, num(ho10["ate_online_rmse_mean"]), 0.00051)
    chk.close("6.9 o20 rgbd ate 0.0755", 0.0755, num(ho20["ate_online_rmse_mean"]), 0.000051)
    chk.close("6.9 o20 ratio 0.59", 0.59,
              num(ho20["ate_online_rmse_mean"]) / num(hc_rgbd["ate_online_rmse_mean"]), 0.0051)
    chk.close("6.9 o20 segs 97.3", 97.3, num(ho20["n_segments_mean"]), 0.051)

    # ---- slides ----
    chk.ok("slides easy runs 81, 2 crashes",
           len(easy) == 81 and len([x for x in easy if x["crashed"].lower() == "true"]) == 2,
           (len(easy), len([x for x in easy if x["crashed"].lower() == "true"])), (81, 2))
    chk.ok("slides 162 runs = 2 seq x 27 x 3",
           len(runs) == 2 * 27 * 3, len(runs), 162)
    chk.ok("slides 7b 12/12 depth_off init", len(doff) == 12 and all(x["status"] == "init_failed" for x in doff),
           len(doff), 12)
    chk.ok("slides 7b IMU_RGBD outage lost 0",
           all(num(chk.row("office_easy_p004", "outage", l, "IMU_RGBD")["lost_events_mean"]) == 0.0
               for l in ("O05", "O10", "O20")), "0", 0)

    # ---- figures referenced by presentation/C ----
    for f in ["01_depth_mosaic_easy.png", "02_invalid_ratio_easy.png", "03_noise_easy.png",
              "04_outage_easy.png", "05_status_easy.png", "06_invalid_ratio_hard.png",
              "07_noise_hard.png", "08_outage_hard.png", "09_status_hard.png"]:
        p = os.path.join(ROOT, "presentation/C/figures", f)
        chk.ok("figure " + f, os.path.isfile(p), os.path.isfile(p), True)
    for ref in ["figures/01_depth_mosaic_easy.png", "figures/04_outage_easy.png",
                "figures/02_invalid_ratio_easy.png", "figures/03_noise_easy.png",
                "figures/05_status_easy.png", "figures/06_invalid_ratio_hard.png"]:
        chk.ok("slides ref " + ref, ref in slides, ref in slides, True)

    # ---- output ----
    text = "checked %d numbers/claims, %d mismatches\n" % (chk.n, len(chk.bad))
    for b in chk.bad:
        text += "MISMATCH " + b + "\n"
    out = os.path.join(ROOT, "results/T7_audit/presentation_c_check.txt")
    open(out, "w").write(text)
    sys.stdout.write(text)


if __name__ == "__main__":
    main()
