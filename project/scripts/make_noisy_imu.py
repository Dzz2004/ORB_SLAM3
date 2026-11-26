#!/usr/bin/env python
"""Write imu_noisy_s<seed>.csv for converted sequences without touching rgb/ or depth/.

Uses the raw noise-free IMU (path from convert_info.json) and the same noise model as
convert_tartanair.py, so seed s here equals --noisy-imu-seed s there.

Usage: python scripts/make_noisy_imu.py data/seqs/office_easy_p004 data/seqs/office_hard_p004 --seeds 0 1 2
"""
import argparse
import json
import os

import numpy as np

from convert_tartanair import add_imu_noise, write_imu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seqs", nargs="+")
    ap.add_argument("--seeds", type=int, nargs="+", required=True)
    a = ap.parse_args()
    for seq in a.seqs:
        info_path = os.path.join(seq, "convert_info.json")
        info = json.load(open(info_path))
        imu_dir = os.path.join(info["source"]["raw_dir"], "imu")
        imu_t = np.round(np.loadtxt(os.path.join(imu_dir, "imu_time.txt")), 6)
        gyro = np.loadtxt(os.path.join(imu_dir, "gyro.txt"))
        acc = np.loadtxt(os.path.join(imu_dir, "acc.txt"))
        for s in a.seeds:
            gn, an = add_imu_noise(gyro, acc, info["imu_hz"], s)
            write_imu(os.path.join(seq, f"imu_noisy_s{s}.csv"), imu_t, gn, an)
            print(seq, f"imu_noisy_s{s}.csv", len(imu_t))
        info["noisy_imu_seeds"] = sorted(set(info.get("noisy_imu_seeds") or
                                             ([info["noisy_imu_seed"]] if info.get("noisy_imu_seed") is not None else []))
                                         | set(a.seeds))
        with open(info_path, "w") as f:
            json.dump(info, f, indent=2)


if __name__ == "__main__":
    main()
