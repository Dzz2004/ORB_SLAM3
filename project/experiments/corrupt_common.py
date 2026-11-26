"""Shared helpers for visual_corrupt.py (B) and depth_corrupt.py (C).

Sequence layout (SETUP.md section 3):
    <seq>/manifest.csv   timestamp,rgb,depth,gt_index (paths relative to <seq>)
    <seq>/rgb/*.png, <seq>/depth/*.png, imu.csv, trajectory_gt.tum, calib*.yaml, ...

A corrupted sequence <out_root>/<seq>__<fault>_<level>_s<seed>/ replaces only one
modality (rgb or depth). Everything else is a relative symlink to the source.
Frames that the fault leaves untouched are symlinked file-by-file as well.
"""

import csv
import hashlib
import json
import os
import shutil
import sys
import time
import zlib
from concurrent.futures import ProcessPoolExecutor

import numpy as np

FAULT_TABLE = "fault_table.csv"
FAULT_META = "fault_meta.json"


def load_manifest(seq_dir):
    path = os.path.join(seq_dir, "manifest.csv")
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty manifest: {path}")
    for col in ("timestamp", "rgb", "depth"):
        if col not in rows[0]:
            raise ValueError(f"{path}: missing column '{col}'")
    for r in rows:
        r["timestamp"] = float(r["timestamp"])
    return rows


def check_rel_path(p):
    """Manifest image paths must stay inside the sequence directory."""
    norm = os.path.normpath(p)
    if os.path.isabs(norm) or norm.startswith(".."):
        raise ValueError(f"manifest path escapes sequence dir: {p}")
    return norm


def rng_for(seed, fault, *keys):
    """Deterministic RNG from (seed, fault name, extra integer keys).

    Uses crc32 instead of hash() so results do not depend on PYTHONHASHSEED.
    """
    words = [int(seed), zlib.crc32(fault.encode())] + [int(k) for k in keys]
    return np.random.default_rng(np.random.SeedSequence(words))


def window_mask(timestamps, start_s, duration_s):
    """Frames with t0+start <= t < t0+start+duration (duration None = to end)."""
    t = np.asarray(timestamps) - timestamps[0]
    start = 0.0 if start_s is None else float(start_s)
    m = t >= start - 1e-9
    if duration_s is not None:
        m &= t < start + float(duration_s) - 1e-9
    return m


def out_name(seq_dir, fault, level, seed):
    base = os.path.basename(os.path.normpath(seq_dir))
    return f"{base}__{fault}_{level}_s{seed}"


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def _relsymlink(target, link):
    # Relative to the link's directory; out_dir and out_dir.partial are siblings,
    # so the links stay valid after the final rename.
    link_dir = os.path.dirname(os.path.abspath(link))
    os.makedirs(link_dir, exist_ok=True)
    os.symlink(os.path.relpath(os.path.realpath(target), link_dir), link)


def link_untouched(src_dir, tmp_dir, modality_dir):
    """Symlink every top-level entry of src_dir except the replaced modality and our own outputs."""
    skip = {modality_dir, FAULT_TABLE, FAULT_META}
    for name in sorted(os.listdir(src_dir)):
        if name in skip or name.startswith("."):
            continue
        _relsymlink(os.path.join(src_dir, name), os.path.join(tmp_dir, name))


def run_frames(fn, jobs, workers):
    """Run fn(job) for each job, in a process pool when workers > 1. Returns results in order."""
    if workers <= 1:
        return [fn(j) for j in jobs]
    with ProcessPoolExecutor(max_workers=workers, initializer=_worker_init) as ex:
        return list(ex.map(fn, jobs, chunksize=max(1, len(jobs) // (workers * 4))))


def _worker_init():
    import cv2
    cv2.setNumThreads(1)


def build_sequence(src_dir, out_dir, modality, process_frame, frame_jobs, table_header,
                   meta, workers, overwrite):
    """Create out_dir atomically.

    modality: manifest column to replace ('rgb' or 'depth').
    process_frame(job) -> (row_dict, wrote_file: bool); writes the corrupted image to
        job['dst'] itself when the fault applies, otherwise returns wrote_file=False and
        the source frame is symlinked.
    """
    if os.path.exists(out_dir):
        if not overwrite:
            print(f"[skip] exists: {out_dir}")
            return False
        shutil.rmtree(out_dir)
    tmp_dir = out_dir + ".partial"
    if os.path.exists(tmp_dir):
        shutil.rmtree(tmp_dir)
    os.makedirs(tmp_dir)

    # The modality directory is taken from the manifest's first path component.
    first = check_rel_path(frame_jobs[0]["rel"])
    modality_dir = first.split(os.sep)[0]
    link_untouched(src_dir, tmp_dir, modality_dir)

    for j in frame_jobs:
        j["dst"] = os.path.join(tmp_dir, check_rel_path(j["rel"]))
        os.makedirs(os.path.dirname(j["dst"]), exist_ok=True)

    t0 = time.time()
    results = run_frames(process_frame, frame_jobs, workers)
    rows = []
    for j, (row, wrote) in zip(frame_jobs, results):
        if not wrote:
            _relsymlink(j["src"], j["dst"])
        rows.append(row)

    with open(os.path.join(tmp_dir, FAULT_TABLE), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=table_header)
        w.writeheader()
        w.writerows(rows)

    meta = dict(meta)
    meta.update({
        "source_sequence": os.path.realpath(src_dir),
        "replaced_modality": modality,
        "n_frames": len(rows),
        "n_frames_applied": int(sum(1 for r in rows if int(r["applied"]))),
        "created_unix": round(time.time(), 3),
        "elapsed_s": round(time.time() - t0, 2),
    })
    with open(os.path.join(tmp_dir, FAULT_META), "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    os.rename(tmp_dir, out_dir)
    print(f"[done] {out_dir}  applied {meta['n_frames_applied']}/{meta['n_frames']} frames "
          f"in {meta['elapsed_s']} s")
    return True


def parse_list(s, cast=str):
    if s is None:
        return None
    return [cast(x) for x in s.split(",") if x != ""]


def cpu_quota():
    """CPUs actually available: cgroup v2 cpu.max quota if set, else affinity count.

    os.cpu_count() reports the host's cores (224 here) even when the container is
    capped (cpu.max = 400000 100000 -> 4 CPUs), so do not use it for sizing pools.
    """
    n = len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else (os.cpu_count() or 1)
    try:
        with open("/sys/fs/cgroup/cpu.max") as f:
            quota, period = f.read().split()[:2]
        if quota != "max":
            n = min(n, max(1, int(int(quota) // int(period))))
    except (OSError, ValueError):
        pass
    return n


def default_workers():
    # Half the quota: the team shares the container. Pass --workers to use the assigned share.
    return max(1, cpu_quota() // 2)


def die(msg):
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(2)
