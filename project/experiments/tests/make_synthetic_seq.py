#!/usr/bin/env python3
"""Build a small synthetic sequence in the unified format, for testing the corrupt scripts.

Textured scene panned over time; depth = tilted plane + boxes, 16-bit PNG, scale 1000/m;
some pixels already invalid (0) to mimic sky/holes. 10 Hz like TartanAir.
"""
import argparse
import os

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--frames", type=int, default=150)
ap.add_argument("--size", type=int, default=640)
a = ap.parse_args()

rng = np.random.default_rng(123)
S = a.size
big = np.zeros((S, S * 3, 3), np.uint8)
big[:] = rng.integers(60, 200, (1, 1, 3), dtype=np.uint8)
for _ in range(400):
    c = tuple(int(x) for x in rng.integers(0, 256, 3))
    x, y = int(rng.integers(0, S * 3)), int(rng.integers(0, S))
    if rng.random() < 0.5:
        cv2.rectangle(big, (x, y), (x + int(rng.integers(8, 60)), y + int(rng.integers(8, 60))), c, -1)
    else:
        cv2.circle(big, (x, y), int(rng.integers(4, 30)), c, -1)
big = cv2.add(big, rng.integers(0, 25, big.shape, dtype=np.uint8))

os.makedirs(os.path.join(a.out, "rgb"), exist_ok=True)
os.makedirs(os.path.join(a.out, "depth"), exist_ok=True)
yy, xx = np.mgrid[0:S, 0:S].astype(np.float32)
lines = ["timestamp,rgb,depth,gt_index"]
gt = []
t0 = 1000.0
for i in range(a.frames):
    t = t0 + i * 0.1
    off = int(i * (S * 2 - 1) / max(1, a.frames - 1))
    rgb = big[:, off:off + S]
    z = 2.0 + 6.0 * yy / S + 1.5 * np.sin((xx + off) / 90.0)
    z[100:220, 200 + (i % 50):330 + (i % 50)] = 1.2
    z[: S // 8] = 0.0                       # "sky" -> invalid already
    d = np.clip(np.rint(z * 1000.0), 0, 65535).astype(np.uint16)
    name = f"{i:06d}.png"
    cv2.imwrite(os.path.join(a.out, "rgb", name), rgb)
    cv2.imwrite(os.path.join(a.out, "depth", name), d)
    lines.append(f"{t:.6f},rgb/{name},depth/{name},{i}")
    gt.append(f"{t:.6f} {off * 0.01:.6f} 0 0 0 0 0 1")
open(os.path.join(a.out, "manifest.csv"), "w").write("\n".join(lines) + "\n")
open(os.path.join(a.out, "trajectory_gt.tum"), "w").write("\n".join(gt) + "\n")
open(os.path.join(a.out, "imu.csv"), "w").write("timestamp,gx,gy,gz,ax,ay,az\n")
open(os.path.join(a.out, "calib_rgbd.yaml"), "w").write(
    "%YAML:1.0\nFile.version: \"1.0\"\nCamera.type: \"PinHole\"\nRGBD.DepthMapFactor: 1000.0\n")
print(f"synthetic sequence: {a.out} ({a.frames} frames)")
