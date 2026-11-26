#!/usr/bin/env python3
"""Checks on corrupted sequences produced from the synthetic sequence.

usage: selftest_corrupt.py <src_seq> <out_root>
"""
import csv
import json
import os
import sys

import cv2
import numpy as np

src, root = sys.argv[1], sys.argv[2]
base = os.path.basename(os.path.normpath(src))
fails = []


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    if not cond:
        fails.append(msg)


def table(d):
    with open(os.path.join(d, "fault_table.csv")) as f:
        return list(csv.DictReader(f))


def seq(name):
    return os.path.join(root, f"{base}__{name}")


# --- structure: replaced modality is real files where applied, everything else symlinks
d = seq("gaussian_blur_L2_s0")
check(os.path.islink(os.path.join(d, "depth")), "B: depth/ is a symlink")
check(os.path.islink(os.path.join(d, "manifest.csv")), "B: manifest.csv is a symlink")
check(not os.path.islink(os.path.join(d, "rgb")) and os.path.isdir(os.path.join(d, "rgb")), "B: rgb/ is a real dir")
check(os.path.realpath(os.path.join(d, "manifest.csv")) == os.path.realpath(os.path.join(src, "manifest.csv")),
      "B: manifest link resolves to source")
check(len(os.listdir(os.path.join(d, "rgb"))) == len(os.listdir(os.path.join(src, "rgb"))), "B: rgb frame count")
check(not os.path.exists(d + ".partial"), "no .partial leftovers")

# --- determinism: s0 run twice identical (second run in *_rerun root), s0 != s1
rr = os.path.join(root + "_rerun", f"{base}__occlusion_L3_s0")
a = cv2.imread(os.path.join(seq("occlusion_L3_s0"), "rgb", "000010.png"))
b = cv2.imread(os.path.join(rr, "rgb", "000010.png"))
c = cv2.imread(os.path.join(seq("occlusion_L3_s1"), "rgb", "000010.png"))
check(a is not None and b is not None and np.array_equal(a, b), "B: same seed -> identical frames")
check(c is not None and not np.array_equal(a, c), "B: different seed -> different frames")

# --- monotonic severity on B (mean over frames)
for f, col in [("gaussian_blur", "lapvar_after"), ("motion_blur", "lapvar_after"),
               ("texture_erase", "lapvar_after"), ("brightness_contrast", "mean_after")]:
    vals = [np.mean([float(r[col]) for r in table(seq(f"{f}_L{k}_s0"))]) for k in range(1, 5)]
    # lapvar saturates at the 8-bit quantisation floor (~1-2) under heavy blur; allow 0.5 slack
    check(all(x > y - 0.5 for x, y in zip(vals, vals[1:])) and vals[0] > vals[-1], f"B: {f} {col} decreases L1->L4 {np.round(vals, 1).tolist()}")
occ = [np.mean([float(r["affected_frac"]) for r in table(seq(f"occlusion_L{k}_s0"))]) for k in range(1, 5)]
check(all(x < y for x, y in zip(occ, occ[1:])), f"B: occlusion area increases {np.round(occ, 3).tolist()}")
orb = [np.mean([float(r["orb_after"]) for r in table(seq(f"gaussian_blur_L{k}_s0"))]) for k in range(1, 5)]
check(orb[0] > orb[-1], f"B: ORB count drops with blur {np.round(orb, 0).tolist()}")

# --- C: invalid ratio matches target, previously invalid stays invalid
for lvl, target in [("P00", 0.0), ("P20", 0.2), ("P50", 0.5), ("P80", 0.8)]:
    t = table(seq(f"invalid_pixels_{lvl}_s0"))
    got = np.mean([float(r["invalidated_frac"]) for r in t])
    check(abs(got - target) < 0.01, f"C: invalid_pixels {lvl} invalidated {got:.4f} ~ {target}")
d0 = cv2.imread(os.path.join(src, "depth", "000020.png"), cv2.IMREAD_UNCHANGED)
d5 = cv2.imread(os.path.join(seq("invalid_pixels_P50_s0"), "depth", "000020.png"), cv2.IMREAD_UNCHANGED)
check(d5.dtype == np.uint16 and d5.shape == d0.shape, "C: output is uint16 same shape")
check(not np.any(d5[d0 == 0]), "C: originally-invalid pixels remain 0")
check(np.array_equal(d5[d5 > 0], d0[d5 > 0]), "C: kept pixels unchanged")
check(os.path.islink(os.path.join(seq("invalid_pixels_P50_s0"), "rgb")), "C: rgb/ symlinked (image kept)")

# --- C: noise rmse increases
rm = [np.mean([float(r["noise_rmse_m"]) for r in table(seq(f"depth_noise_N{k}_s0"))]) for k in range(1, 5)]
check(all(x < y for x, y in zip(rm, rm[1:])), f"C: noise rmse increases {np.round(rm, 4).tolist()}")
meta = json.load(open(os.path.join(seq("depth_noise_N1_s0"), "fault_meta.json")))
if (meta.get("noise_model") or {}).get("model") == "relative":
    for k in range(1, 5):
        r = float(json.load(open(os.path.join(seq(f"depth_noise_N{k}_s0"), "fault_meta.json")))["level_params"]["rel_sigma"])
        got = np.mean([float(x["noise_rel_rmse"]) for x in table(seq(f"depth_noise_N{k}_s0"))])
        # quantisation to 1 mm adds a little at small r; clipping at z'<=0 is negligible for r<=0.1
        check(abs(got - r) < 0.1 * r + 0.002, f"C: relative noise N{k} rel_rmse {got:.4f} ~ r={r}")

# --- C: outage covers exactly duration*10 frames, all-zero, starts at 10 s
for lvl, n in [("O05", 5), ("O10", 10), ("O20", 20)]:
    d = seq(f"outage_{lvl}_s0")
    t = table(d)
    app = [r for r in t if r["applied"] == "1"]
    check(len(app) == n, f"C: outage {lvl} frames applied = {len(app)} (expect {n})")
    check(all(float(r["valid_frac_after"]) == 0 for r in app), f"C: outage {lvl} frames all-zero")
    first = int(app[0]["frame_idx"]) if app else -1
    check(first == 100, f"C: outage {lvl} starts at frame {first} (t0+10s)")
    p = os.path.join(d, "depth", f"{first - 1:06d}.png")
    check(os.path.islink(p), f"C: outage {lvl} frame before window is a symlink")
    meta = json.load(open(os.path.join(d, "fault_meta.json")))
    check(meta["depth_scale"] == 1000.0 and meta["depth_scale_source"].startswith("calib"), "C: depth scale from calib")
off = table(seq("depth_off_ALL_s0"))
check(all(float(r["valid_frac_after"]) == 0 for r in off), "C: depth_off all frames zero")

print(f"\n{len(fails)} failure(s)")
sys.exit(1 if fails else 0)
