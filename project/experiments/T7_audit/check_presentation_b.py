#!/usr/bin/env python
"""T10 item 3 (B): compare numbers in presentation/B with results/B CSVs / metrics.json.

Read-only. Prints one line per checked number; MISMATCH lines are collected at the end.
"""
import csv
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DOC = os.path.join(ROOT, "presentation/B/实现说明.md")


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def r3(v):
    return None if v is None else round(v, 3)


def main():
    rows = list(csv.DictReader(open(os.path.join(ROOT, "results/B/visual_stress.csv"))))
    runs = list(csv.DictReader(open(os.path.join(ROOT, "results/B/visual_stress_runs.csv"))))
    g = {(r["base_sequence"], r["fault"], r["level"], r["mode"]): r for r in rows}
    bad, ok = [], 0

    def chk(label, got, want, tol=0.0005001):
        nonlocal ok
        if got is None or want is None:
            bad.append("%s: parsed=%r expected=%r" % (label, got, want))
            return
        if abs(got - want) > tol:
            bad.append("%s: doc=%s csv=%s" % (label, got, want))
        else:
            ok += 1

    lines = open(DOC).read().splitlines()
    # --- 5.2 coverage table ---
    i = lines.index("### 5.2 覆盖率（easy，mean±std，n=3，来源 `results/B/visual_stress.csv`）")
    j = lines.index("### 5.3 失效转折点（覆盖率均值首次 < 0.5 的档位，easy，来源同上）")
    header_seen = False
    for ln in lines[i:j]:
        if not ln.startswith("|"):
            continue
        cells = [c.strip() for c in ln.strip().strip("|").split("|")]
        if "故障" in cells[0] or set(cells[0]) <= {"-"}:
            if "故障" in cells[0]:
                header_seen = True
            continue
        if not header_seen:
            continue
        fault, mode = cells[0], cells[1]
        for k, lev in enumerate(["L1", "L2", "L3", "L4"]):
            cell = cells[2 + k].replace("*", "")
            want = g.get(("office_easy_p004", fault, lev, mode))
            if want is None:
                bad.append("5.2 no group %s %s %s" % (fault, lev, mode))
                continue
            if "未初始化" in cell:
                chk("5.2 %s %s %s mean" % (fault, mode, lev), 0.0, num(want["coverage_mean"]), 0.0006)
                continue
            m, s = cell.split("±")
            chk("5.2 %s %s %s mean" % (fault, mode, lev), float(m), num(want["coverage_mean"]))
            chk("5.2 %s %s %s std" % (fault, mode, lev), float(s), num(want["coverage_std"]), 0.0006)

    # --- 5.4 L4 table ---
    i = lines.index("### 5.4 失跟、段数与 ATE（easy，L4，来源 `results/B/visual_stress.csv`）")
    j = lines.index("### 5.5 关于 ATE 的解读")
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
        fault, mode = cells[0], cells[1]
        w = g.get(("office_easy_p004", fault, "L4", mode))
        if w is None:
            bad.append("5.4 no group %s %s" % (fault, mode))
            continue
        mapping = [("coverage", "coverage_mean", 3), ("lost_events", "lost_events_mean", 1),
                   ("n_unrecovered", "n_unrecovered_mean", 1), ("n_segments", "n_segments_mean", 1),
                   ("aligned_coverage", "aligned_coverage_mean", 3)]
        for k, (col, csvcol, dec) in enumerate(mapping):
            cell = cells[2 + k]
            exp = num(w.get(csvcol))
            if cell == "-":
                chk("5.4 %s %s %s none" % (fault, mode, col), 1.0 if exp is None else 0.0, 1.0)
                continue
            chk("5.4 %s %s %s" % (fault, mode, col), float(cell), exp, 0.051 if dec <= 1 else 0.0006)
        # online ATE (n)
        ate_cell = cells[7]
        exp = num(w.get("ate_online_rmse_mean"))
        expn = num(w.get("ate_online_rmse_n"))
        if ate_cell == "- (-)":
            chk("5.4 %s %s ate none" % (fault, mode), 1.0 if exp is None else 0.0, 1.0)
        else:
            a, n = ate_cell.replace(" ", "").split("(")
            chk("5.4 %s %s ate" % (fault, mode), float(a), exp)
            chk("5.4 %s %s ate_n" % (fault, mode), float(n.strip(")")), expn, 0.0001)
        fin = cells[8]
        exp = num(w.get("ate_final_rmse_mean"))
        if fin == "-":
            chk("5.4 %s %s final none" % (fault, mode), 1.0 if exp is None else 0.0, 1.0)
        else:
            chk("5.4 %s %s final" % (fault, mode), float(fin), exp)

    # --- 5.1 totals ---
    e = [r for r in runs if r["base_sequence"] == "office_easy_p004"]
    nok = sum(r["status"] == "ok" for r in e)
    nin = sum(r["status"] == "init_failed" for r in e)
    chk("5.1 easy runs", float(len(e)), 189.0, 0.1)
    chk("5.1 easy ok", float(nok), 183.0, 0.1)
    chk("5.1 easy init", float(nin), 6.0, 0.1)

    # --- 5.6 hard counts from metrics.json ---
    hd = os.path.join(ROOT, "results/B/runs")
    nmet = nokh = ninh = 0
    for name in os.listdir(hd):
        if not name.startswith("office_hard"):
            continue
        p = os.path.join(hd, name, "metrics.json")
        if os.path.isfile(p):
            nmet += 1
            st = json.load(open(p)).get("status")
            nokh += st == "ok"
            ninh += st == "init_failed"
    # The doc freezes hard progress at "截至 13:03"; B_hard keeps running, so a larger current count is
    # expected and is reported as drift, not as an inconsistency. A smaller count would be an error.
    print("5.6 hard progress: doc 81 metrics (75 ok + 6 init) as of 13:03; current %d metrics (%d ok + %d init)"
          % (nmet, nokh, ninh))
    if nmet < 81 or nokh < 75 or ninh < 6:
        bad.append("5.6 hard progress went backwards: doc 81/75/6, current %d/%d/%d" % (nmet, nokh, ninh))
    print("checked %d numbers, %d mismatches" % (ok + len(bad), len(bad)))
    for b in bad:
        print("MISMATCH " + b)


if __name__ == "__main__":
    main()
