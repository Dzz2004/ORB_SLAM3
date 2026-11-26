#!/usr/bin/env python
"""Run a B/C experiment matrix: corrupt -> ORB-SLAM3 -> evaluate, in parallel and resumable.

Usage:
    python scripts/run_matrix.py configs/matrix/X.yaml --workers N [--corrupt-workers M] [--cleanup] [--dry-run]
        [--only SUBSTRING] [--rerun-failed] [--max-pending-seqs 16]

The container has a 4-CPU cgroup quota shared by the whole team (os.cpu_count() reports the host's 224),
so --workers has no default: keep the total across concurrent matrices within the CPU quota (4 on the original 4-core container).
--workers is the total number of CPU-bound processes this script runs at once: each ORB-SLAM3 run takes
one slot and each corrupt call takes --corrupt-workers slots (must be <= --workers), from the same pool.
Evaluation runs inside the ORB-SLAM3 run's slot, after ORB-SLAM3 has exited.

Root-disk rule: every subprocess gets TMPDIR/TMP/TEMP, MPLCONFIGDIR, XDG_CACHE_HOME, XDG_CONFIG_HOME and
PIP_CACHE_DIR pointed into <out_dir>/_matrix/env/, and core dumps disabled. HOME is left unchanged (user
rule); evaluate.py only imports evo.core / file_interface, which do not create ~/.evo (checked: no
/root/.evo after all runs).

Matrix yaml (paths relative to ROOT):
    name: B_main
    task: B                      # B -> experiments/B_visual_degradation/visual_corrupt.py
                                 # C -> experiments/C_depth_degradation/depth_corrupt.py
    fault_config: configs/B_visual.yaml   # optional, default = the corrupt script's default
    out_dir: results/B/runs
    sequences: [office_easy_p004, office_hard_p004]
    seeds: [0, 1, 2]
    modes: [MONOCULAR, IMU_MONOCULAR]
    include_clean: true          # also run the unperturbed sequence for every seed x mode
    clean_modes: [IMU_MONOCULAR] # optional extra modes run only on the clean sequence (controls)
    faults:                      # fault name -> levels (null = all levels in fault_config)
      gaussian_blur: [L1, L2, L3, L4]
      motion_blur: null
    run_timeout_s: 1800          # per ORB-SLAM3 run (RUN_TIMEOUT of scripts/run_orbslam.sh)
    runner_args: []              # extra args for the runner, e.g. ["--rate", "1"]
    evaluate_args: []            # extra args for evaluate.py, e.g. ["--recover-err", "0.5"]

For each (sequence, fault, level, seed, mode) one run directory
    <out_dir>/<sequence>__<fault>_<level>_s<seed>__<MODE>/     (clean: <sequence>__clean_s<seed>__<MODE>)
gets: run.log, run_info.json, tracking.csv, trajectory_*.tum   (scripts/run_orbslam.sh)
      metrics.json        evaluate.py on trajectory_online.tum, --segment-by-map   (primary)
      metrics_final.json  evaluate.py on trajectory_est.tum, global alignment       (secondary)
      fault_meta.json, fault_summary.json   copied/summarised from the corrupted sequence
      faults.csv          outage window, passed to evaluate.py --fault-intervals
      matrix_run.json     what was run, exit code, timeout, wall time
IMU modes use SEQ/imu_noisy_s<seed>.csv (IMU_SEED=seed); the seed also seeds the corruption.

A run whose metrics.json exists is skipped (resume). Failed runs (non-zero exit, timeout, no output,
initialisation failure) still get a metrics.json with "status" and are not retried unless
--rerun-failed is given. With --cleanup, a corrupted sequence that this script generated (listed in
<out_dir>/_matrix/created_seqs.txt) is deleted once all its runs are evaluated; sequences that already
existed are never deleted.
"""
import argparse
import csv
import datetime
import json
import os
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SEQS = os.path.join(ROOT, "data", "seqs")
CORRUPT = {
    "B": os.path.join(ROOT, "experiments", "B_visual_degradation", "visual_corrupt.py"),
    "C": os.path.join(ROOT, "experiments", "C_depth_degradation", "depth_corrupt.py"),
}
DEFAULT_FAULT_CONFIG = {"B": "configs/B_visual.yaml", "C": "configs/C_depth.yaml"}
RUNNER = os.path.join(ROOT, "scripts", "run_orbslam.sh")
EVALUATE = os.path.join(ROOT, "scripts", "evaluate.py")
MODES = ["MONOCULAR", "IMU_MONOCULAR", "RGBD", "IMU_RGBD"]

print_lock = threading.Lock()


def log(*a):
    with print_lock:
        print(datetime.datetime.now().strftime("%H:%M:%S"), *a, flush=True)


def rootpath(p):
    return p if os.path.isabs(p) else os.path.join(ROOT, p)


def load_matrix(path):
    with open(path) as f:
        m = yaml.safe_load(f)
    for k in ["name", "task", "out_dir", "sequences", "seeds", "modes"]:
        if k not in m:
            sys.exit(f"matrix {path}: missing key '{k}'")
    if m["task"] not in CORRUPT:
        sys.exit(f"matrix {path}: task must be one of {list(CORRUPT)}")
    m.setdefault("clean_modes", [])
    for mode in m["modes"] + m["clean_modes"]:
        if mode not in MODES:
            sys.exit(f"matrix {path}: bad mode {mode}")
    m.setdefault("faults", {})
    m.setdefault("include_clean", False)
    m.setdefault("run_timeout_s", 1800)
    m.setdefault("runner_args", [])
    m.setdefault("evaluate_args", [])
    m["fault_config"] = rootpath(m.get("fault_config") or DEFAULT_FAULT_CONFIG[m["task"]])
    return m


def expand(m):
    """Return list of corrupted-sequence specs, each with its runs."""
    with open(m["fault_config"]) as f:
        fcfg = yaml.safe_load(f)["faults"]
    specs = []
    for seq in m["sequences"]:
        if not os.path.isfile(os.path.join(SEQS, seq, "manifest.csv")):
            sys.exit(f"sequence not found: {os.path.join(SEQS, seq)}")
        cells = [(None, None, s) for s in m["seeds"]] if (m["include_clean"] or m["clean_modes"]) else []
        for fault, levels in m["faults"].items():
            if fault not in fcfg:
                sys.exit(f"fault '{fault}' not in {m['fault_config']}")
            for level in (levels or list(fcfg[fault]["levels"])):
                if level not in fcfg[fault]["levels"]:
                    sys.exit(f"level '{level}' not in fault '{fault}'")
                cells += [(fault, level, s) for s in m["seeds"]]
        for fault, level, seed in cells:
            name = seq if fault is None else f"{seq}__{fault}_{level}_s{seed}"
            tag = f"clean_s{seed}" if fault is None else f"{fault}_{level}_s{seed}"
            runs = [{"name": f"{seq}__{tag}__{mode}", "mode": mode, "seed": seed, "base": seq,
                     "fault": fault or "clean", "level": level or "", "seq_name": name}
                    for mode in ((m["modes"] if m["include_clean"] else []) + [c for c in m["clean_modes"] if c not in m["modes"]]
                                 if fault is None else m["modes"])]
            specs.append({"seq": seq, "fault": fault, "level": level, "seed": seed, "name": name,
                          "dir": os.path.join(SEQS, name), "runs": runs})
    # Clean sequences are shared by all seeds; merge them into one spec per sequence.
    merged, seen = [], {}
    for s in specs:
        if s["fault"] is None and s["name"] in seen:
            seen[s["name"]]["runs"] += s["runs"]
            continue
        seen[s["name"]] = s
        merged.append(s)
    return merged


def is_done(run_dir, rerun_failed):
    mj = os.path.join(run_dir, "metrics.json")
    if not os.path.isfile(mj):
        return False
    if not rerun_failed:
        return True
    try:
        return json.load(open(mj)).get("status") == "ok"
    except Exception:
        return False


def summarise_fault_table(path):
    """Mean of numeric fault_table.csv columns over all frames and over applied frames."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    out = {"n_frames": len(rows), "n_applied": sum(int(r["applied"]) for r in rows)}
    skip = {"frame_idx", "timestamp", "seed", "applied"}
    for col in (rows[0].keys() if rows else []):
        if col in skip:
            continue
        for sub, rs in [("all", rows), ("applied", [r for r in rows if int(r["applied"])])]:
            vals = []
            for r in rs:
                try:
                    vals.append(float(r[col]))
                except (TypeError, ValueError):
                    pass
            if vals:
                out[f"{col}_mean_{sub}"] = sum(vals) / len(vals)
    return out


def fault_intervals(seq_dir, run_dir):
    """Write faults.csv for an outage window; return its path or None."""
    mp = os.path.join(seq_dir, "fault_meta.json")
    if not os.path.isfile(mp):
        return None
    meta = json.load(open(mp))
    if meta.get("fault_type") != "outage" or meta.get("window_first_ts") is None:
        return None
    if (meta.get("window") or {}).get("duration_s") is None:  # whole-sequence outage: no recovery
        return None
    with open(os.path.join(seq_dir, "manifest.csv")) as f:
        ts = [float(r["timestamp"]) for r in csv.DictReader(f)]
    dt = (ts[-1] - ts[0]) / max(1, len(ts) - 1)
    p = os.path.join(run_dir, "faults.csv")
    with open(p, "w") as f:
        f.write("start,end\n%.6f,%.6f\n" % (meta["window_first_ts"], meta["window_last_ts"] + dt))
    return p


def corrupt(m, spec, workers, py, env):
    cmd = [py, CORRUPT[m["task"]], "apply", "--seq", os.path.join(SEQS, spec["seq"]),
           "--config", m["fault_config"], "--faults", spec["fault"], "--levels", spec["level"],
           "--seeds", str(spec["seed"]), "--out-root", SEQS, "--workers", str(workers)]
    if m["task"] == "B":
        cmd.append("--no-orb")
    t0 = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if r.returncode != 0 or not os.path.isfile(os.path.join(spec["dir"], "manifest.csv")):
        raise RuntimeError(f"corrupt failed for {spec['name']} (exit {r.returncode}):\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
    return time.time() - t0


def evaluate(py, est, gt, trk, mode, out, seed, seq_name, config, extra, args, env):
    cmd = [py, EVALUATE, "--est", est, "--gt", gt, "--tracking", trk, "--mode", mode, "--out", out,
           "--seed", str(seed), "--sequence", seq_name, "--config", config, "--extra", json.dumps(extra)] + args
    return subprocess.run(cmd, capture_output=True, text=True, env=env)


def do_run(m, spec, run, out_dir, py, matrix_path, base_env):
    rd = os.path.join(out_dir, run["name"])
    if os.path.isdir(rd):
        shutil.rmtree(rd)  # stale partial run (no metrics.json) or --rerun-failed
    os.makedirs(rd)
    seq_dir = spec["dir"]
    env = dict(base_env, RUN_TIMEOUT=str(m["run_timeout_s"]))
    env.pop("IMU_SEED", None)
    if run["mode"].startswith("IMU"):
        env["IMU_SEED"] = str(run["seed"])
    t0 = time.time()
    p = subprocess.run(["bash", RUNNER, run["mode"], seq_dir, rd] + [str(x) for x in m["runner_args"]], env=env)
    wall = time.time() - t0
    info = {"matrix": m["name"], "matrix_file": os.path.abspath(matrix_path), "task": m["task"],
            "base_sequence": run["base"], "sequence_dir": seq_dir, "fault": run["fault"], "level": run["level"],
            "seed": run["seed"], "mode": run["mode"],
            "imu_file": os.path.join(seq_dir, f"imu_noisy_s{run['seed']}.csv") if run["mode"].startswith("IMU") else None,
            "runner_args": m["runner_args"], "run_timeout_s": m["run_timeout_s"],
            "exit_code": p.returncode, "timed_out": p.returncode in (124, 137), "wall_s": round(wall, 2)}

    if spec["fault"] is not None:
        for fn in ["fault_meta.json"]:
            if os.path.isfile(os.path.join(seq_dir, fn)):
                shutil.copyfile(os.path.join(seq_dir, fn), os.path.join(rd, fn))
        ft = os.path.join(seq_dir, "fault_table.csv")
        if os.path.isfile(ft):
            with open(os.path.join(rd, "fault_summary.json"), "w") as f:
                json.dump(summarise_fault_table(ft), f, indent=2)
    info["fault_intervals"] = fault_intervals(seq_dir, rd)

    status = evaluate_run(m, spec, run, rd, info, py, matrix_path, base_env)
    log(f"[{status}] {run['name']} exit={p.returncode} {wall:.0f}s")
    return status


def evaluate_run(m, spec, run, rd, info, py, matrix_path, base_env):
    """Evaluate one finished run directory; write metrics.json / metrics_final.json / matrix_run.json.

    Uses only the run directory and the clean base sequence (GT is never corrupted), so it also works
    after --cleanup deleted the corrupted sequence."""
    gt = os.path.join(SEQS, run["base"], "trajectory_gt.tum")
    trk = os.path.join(rd, "tracking.csv")
    fi = info.get("fault_intervals")
    if fi and not os.path.isfile(fi):
        fi = os.path.join(rd, "faults.csv") if os.path.isfile(os.path.join(rd, "faults.csv")) else None
    extra = {k: info[k] for k in ["matrix", "task", "base_sequence", "fault", "level", "mode", "exit_code",
                                  "timed_out", "wall_s"]}
    cfg = f"{os.path.relpath(m['fault_config'], ROOT)}; {os.path.relpath(os.path.abspath(matrix_path), ROOT)}"
    ev_args = [str(x) for x in m["evaluate_args"]] + (["--fault-intervals", fi] if fi else [])
    status = "ok"
    if not os.path.isfile(trk) or os.path.getsize(trk) == 0:
        status = "timeout" if info["timed_out"] else "run_failed"
    else:
        r1 = evaluate(py, os.path.join(rd, "trajectory_online.tum"), gt, trk, run["mode"],
                      os.path.join(rd, "metrics.json"), run["seed"], spec["name"], cfg,
                      dict(extra, trajectory="online"), ev_args + ["--segment-by-map"], base_env)
        r2 = evaluate(py, os.path.join(rd, "trajectory_est.tum"), gt, trk, run["mode"],
                      os.path.join(rd, "metrics_final.json"), run["seed"], spec["name"], cfg,
                      dict(extra, trajectory="final"), ev_args, base_env)
        with open(os.path.join(rd, "evaluate.log"), "w") as f:
            f.write(r1.stdout + r1.stderr + "\n---- final ----\n" + r2.stdout + r2.stderr)
        if r1.returncode != 0:
            status = "eval_failed"
        else:
            mm = json.load(open(os.path.join(rd, "metrics.json")))
            if info["timed_out"]:
                status = "timeout"
            elif info["exit_code"] != 0:
                status = "run_failed"
            elif not mm.get("initialized"):
                status = "init_failed"
    info["status"] = status
    info["evaluated"] = datetime.datetime.now().isoformat(timespec="seconds")
    with open(os.path.join(rd, "matrix_run.json"), "w") as f:
        json.dump(info, f, indent=2)
    mj = os.path.join(rd, "metrics.json")
    if status in ("ok", "init_failed", "timeout", "run_failed") and os.path.isfile(mj):
        mm = json.load(open(mj))
        mm["status"] = status
        mm["matrix_run"] = info
        with open(mj + ".tmp", "w") as f:
            json.dump(mm, f, indent=2)
        os.replace(mj + ".tmp", mj)
    else:
        # No tracking output or evaluation error: record the failure so the run is not silently retried.
        with open(mj, "w") as f:
            json.dump({"status": status, "sequence": spec["name"], "mode": run["mode"], "seed": run["seed"],
                       "initialized": False, "matrix_run": info}, f, indent=2)
    return status


def reevaluate(m, specs, out_dir, a, base_env):
    """Re-run only the evaluation on existing run directories (no ORB-SLAM3, no corruption), one at a time."""
    res = {}
    for s in specs:
        for r in s["runs"]:
            if a.only is not None and a.only not in r["name"]:
                continue
            rd = os.path.join(out_dir, r["name"])
            mr = os.path.join(rd, "matrix_run.json")
            if not os.path.isfile(mr):
                continue
            info = json.load(open(mr))
            st = evaluate_run(m, s, r, rd, info, a.python, a.matrix, base_env)
            res[st] = res.get(st, 0) + 1
            log(f"[re-eval {st}] {r['name']}")
    log(f"re-evaluated: {res}")
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("matrix")
    ap.add_argument("--workers", type=int, default=None,
                    help="parallel ORB-SLAM3 runs; required (keep the total within the CPU quota), not needed for --dry-run")
    ap.add_argument("--corrupt-workers", type=int, default=1,
                    help="processes per corrupt call (counts against the same CPU share)")
    ap.add_argument("--cleanup", action="store_true", help="delete corrupted sequences created by this script")
    ap.add_argument("--max-pending-seqs", type=int, default=16,
                    help="with --cleanup: at most this many generated sequences waiting for runs")
    ap.add_argument("--only", default=None, help="only runs whose name contains this substring")
    ap.add_argument("--rerun-failed", action="store_true", help="redo runs whose metrics.json status != ok")
    ap.add_argument("--dry-run", action="store_true", help="list what would run, do nothing")
    ap.add_argument("--reevaluate", action="store_true",
                    help="only re-run evaluate.py on existing run directories (sequential, 1 CPU process)")
    ap.add_argument("--python", default=sys.executable, help="python for corrupt/evaluate scripts")
    a = ap.parse_args()

    if a.workers is None and not (a.dry_run or a.reevaluate):
        ap.error("--workers is required (keep the total within the CPU quota)")
    m = load_matrix(a.matrix)
    out_dir = rootpath(m["out_dir"])
    specs = expand(m)
    for s in specs:
        s["todo"] = [r for r in s["runs"] if (a.only is None or a.only in r["name"])
                     and not is_done(os.path.join(out_dir, r["name"]), a.rerun_failed)]
    n_all = sum(len(s["runs"]) for s in specs)
    n_todo = sum(len(s["todo"]) for s in specs)
    log(f"matrix {m['name']}: {len(specs)} sequences, {n_all} runs, {n_todo} to do, workers={a.workers}")
    if a.dry_run:
        for s in specs:
            exists = os.path.isfile(os.path.join(s["dir"], "manifest.csv"))
            for r in s["todo"]:
                print(f"  {r['name']}   seq={'exists' if exists else 'GENERATE'}:{s['name']}")
        return

    meta_dir = os.path.join(out_dir, "_matrix")
    os.makedirs(meta_dir, exist_ok=True)
    shutil.copyfile(a.matrix, os.path.join(meta_dir, os.path.basename(a.matrix)))
    ledger = os.path.join(meta_dir, "created_seqs.txt")
    created = set(open(ledger).read().split()) if os.path.isfile(ledger) else set()
    if a.workers is not None and a.corrupt_workers > a.workers:
        ap.error("--corrupt-workers must be <= --workers (both count against the same CPU share)")
    envd = os.path.join(meta_dir, "env")
    base_env = dict(os.environ)
    for k, sub in [("TMPDIR", "tmp"), ("MPLCONFIGDIR", "mpl"), ("XDG_CACHE_HOME", "cache"),
                   ("XDG_CONFIG_HOME", "config"), ("PIP_CACHE_DIR", "pip")]:
        base_env[k] = os.path.join(envd, sub)
        os.makedirs(base_env[k], exist_ok=True)
    base_env["TMP"] = base_env["TEMP"] = base_env["TMPDIR"]
    try:
        import resource
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))  # inherited by all subprocesses
    except (ImportError, ValueError, OSError):
        pass
    if a.reevaluate:
        reevaluate(m, specs, out_dir, a, base_env)
        return

    counts = {}
    lock = threading.Lock()
    pending = threading.Semaphore(a.max_pending_seqs) if a.cleanup else None
    corrupt_errors = []

    def maybe_cleanup(spec):
        if not a.cleanup or spec["fault"] is None:
            return
        d = os.path.realpath(spec["dir"])
        if spec["name"] not in created:
            return
        if os.path.dirname(d) != os.path.realpath(SEQS) or not os.path.isfile(os.path.join(d, "fault_meta.json")):
            log(f"[cleanup-skip] unexpected path {d}")
            return
        shutil.rmtree(d)
        log(f"[cleanup] removed {spec['name']}")

    def finished(spec, fut):
        with lock:
            counts[spec["name"]] -= 1
            last = counts[spec["name"]] == 0
        try:
            fut.result()
        except Exception as e:  # do_run should not raise; log loudly if it does
            log(f"[ERROR] run crashed in run_matrix: {spec['name']}: {e!r}")
        if last:
            maybe_cleanup(spec)
            if pending:
                pending.release()

    t0 = time.time()
    statuses = []
    slots = threading.BoundedSemaphore(a.workers)  # shared by ORB-SLAM3 runs and corrupt calls

    def take(n):
        for _ in range(n):
            slots.acquire()

    def give(n):
        for _ in range(n):
            slots.release()

    def slot_run(s, r):
        take(1)
        try:
            return do_run(m, s, r, out_dir, a.python, a.matrix, base_env)
        finally:
            give(1)

    def submit_runs(ex, s):
        counts[s["name"]] = len(s["todo"])
        for r in s["todo"]:
            fut = ex.submit(slot_run, s, r)
            with lock:
                statuses.append(fut)
            fut.add_done_callback(lambda f, s=s: finished(s, f))

    def prepare(ex, s):
        """Generate the corrupted sequence (holding corrupt_workers slots), then queue its runs."""
        try:
            take(a.corrupt_workers)
            try:
                el = corrupt(m, s, a.corrupt_workers, a.python, base_env)
            finally:
                give(a.corrupt_workers)
        except RuntimeError as e:
            log(f"[corrupt_failed] {s['name']}")
            corrupt_errors.append(str(e))
            for r in s["todo"]:
                rd = os.path.join(out_dir, r["name"])
                os.makedirs(rd, exist_ok=True)
                with open(os.path.join(rd, "metrics.json"), "w") as f:
                    json.dump({"status": "corrupt_failed", "sequence": s["name"], "mode": r["mode"],
                               "seed": r["seed"], "initialized": False, "error": str(e)[-2000:]}, f, indent=2)
            if pending:
                pending.release()
            return
        with lock:
            created.add(s["name"])
            with open(ledger, "a") as f:
                f.write(s["name"] + "\n")
        log(f"[corrupt] {s['name']} {el:.0f}s")
        submit_runs(ex, s)

    # Threads only wait on the slot semaphore, so the pool can be larger than --workers.
    with ThreadPoolExecutor(max(4, 2 * a.workers + 2)) as ex, ThreadPoolExecutor(1) as gen:
        gen_futs = []
        for s in specs:
            if not s["todo"]:
                continue
            if pending:
                pending.acquire()
            if os.path.isfile(os.path.join(s["dir"], "manifest.csv")):
                submit_runs(ex, s)
            elif s["fault"] is None:
                sys.exit(f"missing clean sequence {s['dir']}")
            else:
                gen_futs.append(gen.submit(prepare, ex, s))
        for f in gen_futs:
            f.result()
        while True:  # wait for every queued run (runs are queued from the generator thread too)
            with lock:
                pend = [f for f in statuses if not f.done()]
            if not pend:
                break
            time.sleep(1)
    res = {}
    for f in statuses:
        try:
            k = f.result()
        except Exception:
            k = "crashed"
        res[k] = res.get(k, 0) + 1
    log(f"done in {time.time() - t0:.0f}s: {res}" + (f", corrupt_failed sequences: {len(corrupt_errors)}" if corrupt_errors else ""))
    with open(os.path.join(meta_dir, "last_invocation.json"), "w") as f:
        json.dump({"argv": sys.argv, "finished": datetime.datetime.now().isoformat(), "status_counts": res,
                   "corrupt_errors": corrupt_errors}, f, indent=2)


if __name__ == "__main__":
    main()
