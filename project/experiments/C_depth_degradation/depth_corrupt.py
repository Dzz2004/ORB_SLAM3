#!/usr/bin/env python3
"""C: offline depth degradation of a unified-format sequence (depth only, RGB kept).

Faults (configs/C_depth.yaml): invalid_pixels (0/20/50/80%), depth_noise
(gaussian, >=3 levels), outage (whole-frame depth = 0 for 0.5/1/2 s), depth_off.
Invalid depth = 0, matching ORB-SLAM3 (Frame::ComputeStereoFromRGBD uses d > 0 only).

Depth PNGs are 16-bit; metres = raw / depth_scale (ORB-SLAM3 RGBD.DepthMapFactor).

  apply   write <out-root>/<seq>__<fault>_<level>_s<seed>/ (depth/ replaced,
          everything else symlinked) + fault_table.csv
  mosaic  original vs. each level for one frame (colour-mapped, invalid = black)

Examples:
  python depth_corrupt.py apply  --seq data/seqs/X --out-root data/seqs
  python depth_corrupt.py apply  --seq data/seqs/X --faults outage --levels O10 --seeds 0
  python depth_corrupt.py mosaic --seq data/seqs/X --frame 100 --out results/C/figures/mosaic.png
"""

import argparse
import glob
import json
import os
import re
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import corrupt_common as cc  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DEFAULT_CONFIG = os.path.join(ROOT, "configs", "C_depth.yaml")

TABLE_HEADER = [
    "frame_idx", "timestamp", "fault", "level", "seed", "applied", "params",
    "valid_frac_before", "valid_frac_after", "invalidated_frac", "noise_rmse_m",
    "noise_rel_rmse", "median_depth_m",
]


# ---------------------------------------------------------------- depth scale

def scale_from_calib(seq_dir):
    """Read RGBD.DepthMapFactor (new settings) or DepthMapFactor (old) from calib*.yaml."""
    found = {}
    for path in sorted(glob.glob(os.path.join(seq_dir, "calib*.yaml"))):
        with open(path, errors="replace") as f:
            for line in f:
                m = re.match(r"\s*(?:RGBD\.)?DepthMapFactor\s*:\s*([-+0-9.eE]+)", line)
                if m:
                    found[os.path.basename(path)] = float(m.group(1))
                    break
    vals = set(found.values())
    if len(vals) > 1:
        cc.die(f"inconsistent DepthMapFactor in calib files: {found}")
    return (vals.pop(), found) if vals else (None, found)


def resolve_scale(args, cfg):
    if args.depth_scale is not None:
        return float(args.depth_scale), "cli"
    if cfg.get("depth_scale") is not None:
        return float(cfg["depth_scale"]), "config"
    s, found = scale_from_calib(args.seq)
    if s is None:
        cc.die("depth scale unknown: no DepthMapFactor in calib*.yaml; pass --depth-scale")
    return s, f"calib:{','.join(found)}"


# ---------------------------------------------------------------- fault primitives

def read_depth(path):
    d = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if d is None:
        raise IOError(f"cannot read depth: {path}")
    if d.ndim != 2 or d.dtype != np.uint16:
        raise ValueError(f"{path}: expected single-channel 16-bit depth PNG, got "
                         f"{d.dtype} shape {d.shape}")
    return d


def noise_sigma(fcfg, p, z):
    """Per-pixel std (m). model 'relative': sigma = rel_sigma * z;
    'quadratic': sigma = sigma0_m + k_quad * z^2."""
    model = fcfg.get("model", "quadratic")
    if model == "relative":
        return float(p["rel_sigma"]) * z
    if model == "quadratic":
        return float(p["sigma0_m"]) + float(p.get("k_quad", 0.0)) * z * z
    raise ValueError(f"unknown noise model: {model}")


NOISE_MODEL_DESC = {
    "relative": "z' = z + N(0, (rel_sigma*z)^2) per valid pixel, independent per pixel and frame",
    "quadratic": "z' = z + N(0, (sigma0_m + k_quad*z^2)^2) per valid pixel, independent per pixel and frame",
}


def apply_fault(raw, fcfg, lp, seed, fault, frame_idx, scale):
    """raw: uint16 depth. Returns (uint16 corrupted, noise_rmse_m or '', params used)."""
    ftype = fcfg["type"]
    p = dict(lp)
    valid = raw > 0
    rng = cc.rng_for(seed, fault, frame_idx)

    if ftype == "invalid_pixels":
        ratio = float(p["ratio"])
        out = raw.copy()
        if ratio <= 0:
            return out, "", p
        if fcfg.get("pattern", "pixel") == "block":
            b = int(fcfg.get("block_px", 8))
            h, w = raw.shape
            hb, wb = -(-h // b), -(-w // b)
            drop = rng.random((hb, wb)) < ratio
            drop = np.kron(drop, np.ones((b, b), bool))[:h, :w]
        else:
            drop = rng.random(raw.shape) < ratio
        out[drop & valid] = 0
        return out, "", p

    if ftype == "gaussian_noise":
        z = raw.astype(np.float64) / scale
        sigma = noise_sigma(fcfg, p, z)
        noise = rng.standard_normal(raw.shape) * sigma
        zn = np.where(valid, z + noise, 0.0)
        maxd = fcfg.get("max_depth_m")
        zn[zn <= 0] = 0.0
        if maxd is not None:
            zn[zn > float(maxd)] = 0.0
        q = np.rint(zn * scale)
        q = np.clip(q, 0, 65535).astype(np.uint16)
        keep = valid & (q > 0)
        err = (q[keep].astype(np.float64) / scale - z[keep])
        rmse = float(np.sqrt(np.mean(err ** 2))) if err.size else 0.0
        p["_rel_rmse"] = round(float(np.sqrt(np.mean((err / z[keep]) ** 2))), 5) if err.size else 0.0
        return q, round(rmse, 5), p

    if ftype == "outage":
        return np.zeros_like(raw), "", p

    raise ValueError(f"unknown fault type: {ftype}")


def fault_window(fcfg, lp, args, timestamps):
    """Outage window comes from start_s + level duration_s; others from optional start/duration."""
    start = args.start_s if args.start_s is not None else fcfg.get("start_s")
    if fcfg["type"] == "outage":
        dur = lp.get("duration_s")
    else:
        dur = args.duration_s if args.duration_s is not None else fcfg.get("duration_s")
    return cc.window_mask(timestamps, start, dur), {"start_s": start, "duration_s": dur}


# ---------------------------------------------------------------- per-frame worker

def process_frame(job):
    raw = read_depth(job["src"])
    scale = job["scale"]
    valid0 = raw > 0
    row = {k: job[k] for k in ("frame_idx", "timestamp", "fault", "level", "seed")}
    row["applied"] = int(job["in_window"])
    row["valid_frac_before"] = round(float(valid0.mean()), 5)
    row["median_depth_m"] = round(float(np.median(raw[valid0]) / scale), 4) if valid0.any() else ""
    if not job["in_window"]:
        row.update(params="", valid_frac_after=row["valid_frac_before"], invalidated_frac=0.0,
                   noise_rmse_m="", noise_rel_rmse="")
        return row, False
    out, rmse, used = apply_fault(raw, job["fcfg"], job["lparams"], job["seed"], job["fault"],
                                  job["frame_idx"], scale)
    v1 = out > 0
    row["noise_rel_rmse"] = used.pop("_rel_rmse", "")
    row["params"] = json.dumps(used, sort_keys=True)
    row["valid_frac_after"] = round(float(v1.mean()), 5)
    row["invalidated_frac"] = round(float((valid0 & ~v1).sum() / max(1, valid0.sum())), 5)
    row["noise_rmse_m"] = rmse
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
    scale, scale_src = resolve_scale(args, cfg)
    rows = cc.load_manifest(args.seq)
    ts = [r["timestamp"] for r in rows]
    out_root = args.out_root or os.path.dirname(os.path.normpath(args.seq))
    n_made = 0
    for fault in faults:
        fcfg = cfg["faults"][fault]
        levels = cc.parse_list(args.levels) or list(fcfg["levels"])
        for level in levels:
            if level not in fcfg["levels"]:
                cc.die(f"level '{level}' not in fault '{fault}'")
            lp = fcfg["levels"][level]
            win, wdesc = fault_window(fcfg, lp, args, ts)
            if fcfg["type"] == "outage" and not win.any():
                print(f"[warn] {fault}/{level}: window {wdesc} contains no frames", file=sys.stderr)
            for seed in seeds:
                out_dir = os.path.join(out_root, cc.out_name(args.seq, fault, level, seed))
                if args.dry_run:
                    print(f"[dry-run] {out_dir}  ({int(win.sum())}/{len(rows)} frames)")
                    continue
                jobs = [{
                    "frame_idx": i, "timestamp": r["timestamp"], "fault": fault, "level": level,
                    "seed": seed, "in_window": bool(win[i]), "rel": r["depth"],
                    "src": os.path.join(args.seq, cc.check_rel_path(r["depth"])),
                    "fcfg": fcfg, "lparams": lp, "scale": scale,
                    "png_compression": int(cfg.get("png_compression", 3)),
                } for i, r in enumerate(rows)]
                in_win_ts = [ts[i] for i in np.flatnonzero(win)]
                meta = {
                    "task": "C_depth_degradation", "fault": fault, "fault_type": fcfg["type"],
                    "level": level, "seed": seed, "level_params": lp,
                    "fault_config": {k: v for k, v in fcfg.items() if k != "levels"},
                    "config_file": os.path.realpath(args.config),
                    "config_version": cfg.get("version"),
                    "config_sha256": cc.file_sha256(args.config),
                    "depth_scale": scale, "depth_scale_source": scale_src,
                    "invalid_value": 0,
                    "noise_model": ({"model": fcfg.get("model", "quadratic"),
                                     "formula": NOISE_MODEL_DESC[fcfg.get("model", "quadratic")],
                                     "params": lp,
                                     "post": "z'<=0 -> 0 (invalid); max_depth_m=%s; quantised to "
                                             "uint16 at depth_scale, clipped to 65535"
                                             % fcfg.get("max_depth_m")}
                                    if fcfg["type"] == "gaussian_noise" else None),
                    "window": wdesc,
                    "window_first_ts": in_win_ts[0] if in_win_ts else None,
                    "window_last_ts": in_win_ts[-1] if in_win_ts else None,
                    "note": "rgb/ unchanged (symlinked); only depth/ is corrupted",
                }
                if cc.build_sequence(args.seq, out_dir, "depth", process_frame, jobs, TABLE_HEADER,
                                     meta, args.workers, args.overwrite):
                    n_made += 1
    print(f"created {n_made} sequence(s)")


def colorize(raw, scale, vmax_m):
    z = raw.astype(np.float32) / scale
    u8 = np.clip(z / vmax_m * 255.0, 0, 255).astype(np.uint8)
    vis = cv2.applyColorMap(255 - u8, cv2.COLORMAP_TURBO)  # near = red, far = blue
    vis[raw == 0] = 0
    return vis


def label(img, text):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(out, text, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def colorbar(height, vmax_m, width=70):
    bar = np.linspace(255, 0, height).astype(np.uint8)[:, None].repeat(22, 1)
    bar = cv2.applyColorMap(255 - bar, cv2.COLORMAP_TURBO)
    out = np.zeros((height, width, 3), np.uint8)
    out[:, 4:26] = bar
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = int(round((1 - frac) * (height - 1)))
        cv2.putText(out, f"{frac * vmax_m:.0f}m", (28, min(height - 4, max(12, y + 4))),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def cmd_mosaic(args):
    cfg = load_config(args.config)
    faults, seeds = select(cfg, args)
    seed = seeds[0]
    scale, _ = resolve_scale(args, cfg)
    rows = cc.load_manifest(args.seq)
    i = args.frame if args.frame is not None else len(rows) // 2
    r = rows[i]
    raw = read_depth(os.path.join(args.seq, cc.check_rel_path(r["depth"])))
    v = raw[raw > 0]
    vmax = args.vmax_m or (float(np.percentile(v, 98)) / scale if v.size else 10.0)
    tile = args.tile
    rgb = cv2.imread(os.path.join(args.seq, cc.check_rel_path(r["rgb"])), cv2.IMREAD_COLOR)

    def prep(d, text):
        im = cv2.resize(colorize(d, scale, vmax), (tile, tile), interpolation=cv2.INTER_NEAREST)
        return label(im, f"{text}  valid={float((d > 0).mean()) * 100:.0f}%")

    grid = []
    for fault in faults:
        fcfg = cfg["faults"][fault]
        line = []
        if rgb is not None:
            line.append(label(cv2.resize(rgb, (tile, tile), interpolation=cv2.INTER_AREA), "rgb (kept)"))
        line.append(prep(raw, f"{fault}: original"))
        for level, lp in fcfg["levels"].items():
            out, rmse, used = apply_fault(raw, fcfg, lp, seed, fault, i, scale)
            extra = f" rmse={rmse:.3f}m" if rmse != "" else ""
            if "rel_sigma" in used:
                extra = f" r={float(used['rel_sigma']) * 100:g}%" + extra
            line.append(prep(out, level + extra))
        grid.append(np.hstack(line))
    width = max(g.shape[1] for g in grid)
    grid = [np.pad(g, ((0, 0), (0, width - g.shape[1]), (0, 0))) for g in grid]
    body = np.vstack(grid)
    body = np.hstack([body, colorbar(body.shape[0], vmax)])
    header = np.zeros((34, body.shape[1], 3), np.uint8)
    cv2.putText(header, f"C depth degradation  seq={os.path.basename(os.path.normpath(args.seq))}  "
                f"frame={i}  t={r['timestamp']:.3f}s  seed={seed}  black = invalid (0)", (8, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    mosaic = np.vstack([header, body])
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
        p.add_argument("--depth-scale", type=float, help="raw units per metre (overrides config/calib)")

    a = sub.add_parser("apply")
    common(a)
    a.add_argument("--levels", help="comma list; default all")
    a.add_argument("--out-root", help="default: parent dir of --seq")
    a.add_argument("--start-s", type=float, help="override fault start (s from first frame)")
    a.add_argument("--duration-s", type=float, help="override window length for non-outage faults")
    a.add_argument("--workers", type=int, default=cc.default_workers())
    a.add_argument("--overwrite", action="store_true")
    a.add_argument("--dry-run", action="store_true")
    a.set_defaults(fn=cmd_apply)

    m = sub.add_parser("mosaic")
    common(m)
    m.add_argument("--frame", type=int, help="frame index; default middle frame")
    m.add_argument("--vmax-m", type=float, help="colour scale max (m); default 98th percentile")
    m.add_argument("--tile", type=int, default=320)
    m.add_argument("--out", required=True)
    m.set_defaults(fn=cmd_mosaic)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
