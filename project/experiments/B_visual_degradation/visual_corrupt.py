#!/usr/bin/env python3
"""B: offline visual degradation of a unified-format sequence (RGB only).

Faults (configs/B_visual.yaml): gaussian_blur, motion_blur, brightness_contrast,
occlusion, texture_erase; 4 levels each; deterministic per seed.

  apply   write <out-root>/<seq>__<fault>_<level>_s<seed>/ for every selected
          fault x level x seed; rgb/ holds the corrupted frames, all other files
          are symlinks to the source; fault_table.csv has one row per frame.
  mosaic  original vs. each level for one frame, optionally with ORB keypoints.

Examples:
  python visual_corrupt.py apply  --seq data/seqs/X --config configs/B_visual.yaml --out-root data/seqs
  python visual_corrupt.py apply  --seq data/seqs/X --faults motion_blur --levels L2 --seeds 0
  python visual_corrupt.py mosaic --seq data/seqs/X --frame 100 --orb --out results/B/figures/mosaic.png
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import corrupt_common as cc  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DEFAULT_CONFIG = os.path.join(ROOT, "configs", "B_visual.yaml")

TABLE_HEADER = [
    "frame_idx", "timestamp", "fault", "level", "seed", "applied", "params",
    "affected_frac", "mean_before", "mean_after", "lapvar_before", "lapvar_after",
    "orb_before", "orb_after",
]

ORB_NFEATURES = 1000  # ORB-SLAM3 default ORBextractor.nFeatures for TUM/EuRoC configs


# ---------------------------------------------------------------- fault primitives

def epoch_of(t_rel, refresh_s):
    return 0 if not refresh_s else int(math.floor(t_rel / float(refresh_s) + 1e-9))


def random_rects(h, w, area_frac, n, rng):
    """n axis-aligned rectangles with combined nominal area area_frac*h*w.

    Positions/aspects come from rng before the size is used, so for a given
    rng state every level shares the same centres and only the size changes.
    """
    centres = rng.uniform(0.0, 1.0, size=(n, 2))
    aspects = np.exp(rng.uniform(math.log(0.5), math.log(2.0), size=n))
    rects = []
    a_each = area_frac * h * w / n
    for (cy, cx), asp in zip(centres, aspects):
        rh = min(h, math.sqrt(a_each * asp))
        rw = min(w, a_each / max(rh, 1.0))
        y0 = int(round(np.clip(cy * h - rh / 2, 0, h - rh)))
        x0 = int(round(np.clip(cx * w - rw / 2, 0, w - rw)))
        rects.append((y0, x0, int(round(rh)), int(round(rw))))
    return rects


def rect_mask(h, w, rects):
    m = np.zeros((h, w), np.float32)
    for y0, x0, rh, rw in rects:
        m[y0:y0 + rh, x0:x0 + rw] = 1.0
    return m


def motion_kernel(length, angle_deg):
    length = max(1, int(length))
    if length == 1:
        return np.ones((1, 1), np.float32)
    size = length if length % 2 == 1 else length + 1
    k = np.zeros((size, size), np.float32)
    c = (size - 1) / 2.0
    a = math.radians(angle_deg)
    dx, dy = math.cos(a), -math.sin(a)
    # anti-aliased line by dense sampling
    for s in np.linspace(-(length - 1) / 2.0, (length - 1) / 2.0, length * 4):
        x, y = c + s * dx, c + s * dy
        x0, y0 = int(math.floor(x)), int(math.floor(y))
        fx, fy = x - x0, y - y0
        for yy, xx, wgt in ((y0, x0, (1 - fx) * (1 - fy)), (y0, x0 + 1, fx * (1 - fy)),
                            (y0 + 1, x0, (1 - fx) * fy), (y0 + 1, x0 + 1, fx * fy)):
            if 0 <= yy < size and 0 <= xx < size:
                k[yy, xx] += wgt
    return k / k.sum()


def apply_fault(img, fcfg, level_params, seed, fault_name, frame_idx, t_rel):
    """Return (corrupted uint8 image, affected_frac, params actually used)."""
    ftype = fcfg["type"]
    h, w = img.shape[:2]
    p = dict(level_params)

    if ftype == "gaussian_blur":
        s = float(p["sigma_px"])
        out = cv2.GaussianBlur(img, (0, 0), sigmaX=s, sigmaY=s, borderType=cv2.BORDER_REFLECT)
        return out, 1.0, p

    if ftype == "motion_blur":
        base = fcfg.get("angle_deg")
        if base is None:
            base = float(cc.rng_for(seed, fault_name, 2_000_000_000).uniform(0.0, 180.0))
        jit = float(fcfg.get("angle_jitter_deg", 0.0))
        ang = base + (cc.rng_for(seed, fault_name, frame_idx).uniform(-jit, jit) if jit else 0.0)
        k = motion_kernel(p["length_px"], ang)
        out = cv2.filter2D(img, -1, k, borderType=cv2.BORDER_REFLECT)
        p["angle_deg"] = round(float(ang), 2)
        return out, 1.0, p

    if ftype == "brightness_contrast":
        rng = cc.rng_for(seed, fault_name, frame_idx)
        flick = float(rng.normal(0.0, float(p.get("flicker_std", 0.0)))) if p.get("flicker_std") else 0.0
        x = img.astype(np.float32) / 255.0
        x = np.power(x, float(p.get("gamma", 1.0)))
        x = (x - 0.5) * float(p.get("contrast", 1.0)) + 0.5 + (float(p.get("brightness", 0.0)) + flick) / 255.0
        out = np.clip(np.rint(x * 255.0), 0, 255).astype(np.uint8)
        p["flicker"] = round(flick, 2)
        return out, 1.0, p

    if ftype in ("occlusion", "texture_erase"):
        ep = epoch_of(t_rel, fcfg.get("refresh_s"))
        rng = cc.rng_for(seed, fault_name, 1_000_000 + ep)
        n = int(fcfg.get("n_blobs", fcfg.get("n_regions", 1)))
        m = rect_mask(h, w, random_rects(h, w, float(p["area_frac"]), n, rng))
        p["epoch"] = ep
        if ftype == "occlusion":
            fill = fcfg.get("fill", "black")
            if fill == "black":
                filler = np.zeros_like(img)
            elif fill == "gray":
                filler = np.full_like(img, 128)
            elif fill == "noise":
                filler = cc.rng_for(seed, fault_name, frame_idx).integers(0, 256, img.shape, dtype=np.uint8)
            else:
                raise ValueError(f"unknown occlusion fill: {fill}")
            sel = m.astype(bool)
            out = img.copy()
            out[sel] = filler[sel]
            return out, float(m.mean()), p
        # texture_erase: feathered blend towards a heavily blurred copy
        s = float(fcfg.get("erase_sigma_px", 25.0))
        flat = cv2.GaussianBlur(img, (0, 0), sigmaX=s, sigmaY=s, borderType=cv2.BORDER_REFLECT)
        fe = float(fcfg.get("feather_px", 0.0))
        if fe > 0:
            m = cv2.GaussianBlur(m, (0, 0), sigmaX=fe, sigmaY=fe)
        mm = m[..., None] if img.ndim == 3 else m
        out = np.clip(np.rint(img.astype(np.float32) * (1 - mm) + flat.astype(np.float32) * mm),
                      0, 255).astype(np.uint8)
        return out, float((m > 0.5).mean()), p

    raise ValueError(f"unknown fault type: {ftype}")


# ---------------------------------------------------------------- quality stats

def gray(img):
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def lapvar(g):
    return float(cv2.Laplacian(g, cv2.CV_64F).var())


_ORB = None


def orb_count(g):
    global _ORB
    if _ORB is None:
        _ORB = cv2.ORB_create(nfeatures=ORB_NFEATURES)
    return len(_ORB.detect(g, None))


# ---------------------------------------------------------------- per-frame worker

def read_rgb(path):
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise IOError(f"cannot read image: {path}")
    if img.dtype != np.uint8:
        raise ValueError(f"{path}: expected 8-bit image, got {img.dtype}")
    if img.ndim == 3 and img.shape[2] == 4:
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def process_frame(job):
    import json
    in_win = job["in_window"]
    row = {k: job[k] for k in ("frame_idx", "timestamp", "fault", "level", "seed")}
    row["applied"] = int(in_win)
    img = read_rgb(job["src"])
    g0 = gray(img)
    row["mean_before"] = round(float(g0.mean()), 3)
    row["lapvar_before"] = round(lapvar(g0), 3)
    row["orb_before"] = orb_count(g0) if job["orb"] else ""
    if not in_win:
        row.update(params="", affected_frac=0.0, mean_after=row["mean_before"],
                   lapvar_after=row["lapvar_before"], orb_after=row["orb_before"])
        return row, False
    out, frac, used = apply_fault(img, job["fcfg"], job["lparams"], job["seed"], job["fault"],
                                  job["frame_idx"], job["t_rel"])
    g1 = gray(out)
    row["params"] = json.dumps(used, sort_keys=True)
    row["affected_frac"] = round(frac, 4)
    row["mean_after"] = round(float(g1.mean()), 3)
    row["lapvar_after"] = round(lapvar(g1), 3)
    row["orb_after"] = orb_count(g1) if job["orb"] else ""
    if not cv2.imwrite(job["dst"], out, [cv2.IMWRITE_PNG_COMPRESSION, job["png_compression"]]):
        raise IOError(f"cannot write {job['dst']}")
    return row, True


# ---------------------------------------------------------------- commands

def load_config(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    for name, fc in cfg["faults"].items():
        if "type" not in fc or "levels" not in fc:
            raise ValueError(f"fault '{name}' needs 'type' and 'levels'")
    return cfg


def select(cfg, args):
    faults = cc.parse_list(args.faults) or list(cfg["faults"])
    for f in faults:
        if f not in cfg["faults"]:
            cc.die(f"fault '{f}' not in config (have: {', '.join(cfg['faults'])})")
    seeds = cc.parse_list(args.seeds, int) if args.seeds else list(cfg.get("seeds", [0]))
    return faults, seeds


def cmd_apply(args):
    cfg = load_config(args.config)
    faults, seeds = select(cfg, args)
    rows = cc.load_manifest(args.seq)
    ts = [r["timestamp"] for r in rows]
    t_rel = np.asarray(ts) - ts[0]
    out_root = args.out_root or os.path.dirname(os.path.normpath(args.seq))
    n_made = 0
    for fault in faults:
        fcfg = cfg["faults"][fault]
        levels = cc.parse_list(args.levels) or list(fcfg["levels"])
        win = cc.window_mask(ts, args.start_s if args.start_s is not None else fcfg.get("start_s"),
                             args.duration_s if args.duration_s is not None else fcfg.get("duration_s"))
        for level in levels:
            if level not in fcfg["levels"]:
                cc.die(f"level '{level}' not in fault '{fault}'")
            for seed in seeds:
                out_dir = os.path.join(out_root, cc.out_name(args.seq, fault, level, seed))
                if args.dry_run:
                    print(f"[dry-run] {out_dir}  ({int(win.sum())}/{len(rows)} frames)")
                    continue
                jobs = [{
                    "frame_idx": i, "timestamp": r["timestamp"], "t_rel": float(t_rel[i]),
                    "fault": fault, "level": level, "seed": seed, "in_window": bool(win[i]),
                    "rel": r["rgb"], "src": os.path.join(args.seq, cc.check_rel_path(r["rgb"])),
                    "fcfg": fcfg, "lparams": fcfg["levels"][level], "orb": not args.no_orb,
                    "png_compression": int(cfg.get("png_compression", 3)),
                } for i, r in enumerate(rows)]
                meta = {
                    "task": "B_visual_degradation", "fault": fault, "fault_type": fcfg["type"],
                    "level": level, "seed": seed, "level_params": fcfg["levels"][level],
                    "fault_config": {k: v for k, v in fcfg.items() if k != "levels"},
                    "config_file": os.path.realpath(args.config),
                    "config_version": cfg.get("version"),
                    "config_sha256": cc.file_sha256(args.config),
                    "window": {"start_s": args.start_s if args.start_s is not None else fcfg.get("start_s"),
                               "duration_s": args.duration_s if args.duration_s is not None
                               else fcfg.get("duration_s")},
                    "note": ("artificial image degradation applied offline to recorded frames; "
                             "not a re-rendered sequence")
                            + ("; texture_erase is an image-level approximation, not real "
                               "textureless geometry" if fcfg["type"] == "texture_erase" else ""),
                }
                if cc.build_sequence(args.seq, out_dir, "rgb", process_frame, jobs, TABLE_HEADER,
                                     meta, args.workers, args.overwrite):
                    n_made += 1
    print(f"created {n_made} sequence(s)")


def label(img, text, scale=0.6):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(out, text, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def draw_orb(img):
    g = gray(img)
    kps = cv2.ORB_create(nfeatures=ORB_NFEATURES).detect(g, None)
    vis = img if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return cv2.drawKeypoints(vis, kps, None, color=(0, 255, 0)), len(kps)


def cmd_mosaic(args):
    cfg = load_config(args.config)
    faults, seeds = select(cfg, args)
    seed = seeds[0]
    rows = cc.load_manifest(args.seq)
    i = args.frame if args.frame is not None else len(rows) // 2
    r = rows[i]
    img = read_rgb(os.path.join(args.seq, cc.check_rel_path(r["rgb"])))
    t_rel = r["timestamp"] - rows[0]["timestamp"]
    tile = args.tile

    def prep(im, text):
        n = None
        if args.orb:
            im, n = draw_orb(im)
        if im.ndim == 2:
            im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
        im = cv2.resize(im, (tile, tile), interpolation=cv2.INTER_AREA)
        return label(im, text + (f"  ORB={n}" if n is not None else ""))

    grid = []
    for fault in faults:
        fcfg = cfg["faults"][fault]
        line = [prep(img, f"{fault}: original")]
        for level, lp in fcfg["levels"].items():
            out, _, used = apply_fault(img, fcfg, lp, seed, fault, i, t_rel)
            line.append(prep(out, f"{level}"))
        grid.append(np.hstack(line))
    width = max(g.shape[1] for g in grid)
    grid = [np.pad(g, ((0, 0), (0, width - g.shape[1]), (0, 0))) for g in grid]
    mosaic = np.vstack(grid)
    header = np.zeros((34, width, 3), np.uint8)
    cv2.putText(header, f"B visual degradation  seq={os.path.basename(os.path.normpath(args.seq))}  "
                f"frame={i}  t={r['timestamp']:.3f}s  seed={seed}", (8, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    mosaic = np.vstack([header, mosaic])
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    if not cv2.imwrite(args.out, mosaic):
        cc.die(f"cannot write {args.out}")
    print(f"[mosaic] {args.out}  ({mosaic.shape[1]}x{mosaic.shape[0]})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--seq", required=True, help="source sequence dir (with manifest.csv)")
        p.add_argument("--config", default=DEFAULT_CONFIG)
        p.add_argument("--faults", help="comma list; default all in config")
        p.add_argument("--seeds", help="comma list; default config 'seeds' (mosaic uses the first)")

    a = sub.add_parser("apply")
    common(a)
    a.add_argument("--levels", help="comma list, e.g. L1,L4; default all")
    a.add_argument("--out-root", help="default: parent dir of --seq")
    a.add_argument("--start-s", type=float, help="override fault window start (s from first frame)")
    a.add_argument("--duration-s", type=float, help="override fault window length (s)")
    a.add_argument("--workers", type=int, default=cc.default_workers())
    a.add_argument("--no-orb", action="store_true", help="skip ORB keypoint counts in fault table")
    a.add_argument("--overwrite", action="store_true")
    a.add_argument("--dry-run", action="store_true")
    a.set_defaults(fn=cmd_apply)

    m = sub.add_parser("mosaic")
    common(m)
    m.add_argument("--frame", type=int, help="frame index; default middle frame")
    m.add_argument("--orb", action="store_true", help="draw ORB keypoints and counts")
    m.add_argument("--tile", type=int, default=320)
    m.add_argument("--out", required=True)
    m.set_defaults(fn=cmd_mosaic)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
