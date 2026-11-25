"""Deterministic interface fixtures. These traces are not ORB-SLAM3 measurements."""

import numpy as np
from scipy.spatial.transform import Rotation

from degradation_policy import Pose
from replay import write_csv, write_tum


CASES = {"rgb_low": (4.0, 5.2), "depth_loss": (4.0, 5.5), "rgb_depth_outage": (4.0, 6.8)}
SEEDS = (0, 1, 2)


def truth(time):
    position = np.array([0.55 * time + 0.22 * np.sin(time), 0.32 * np.cos(0.7 * time), 0.04 * time])
    rotation = Rotation.from_euler("z", 0.18 * time + 0.12 * np.sin(0.8 * time))
    return Pose(position, rotation)


def make_case(folder, fault, seed):
    folder.mkdir(parents=True, exist_ok=True)
    start, end = CASES[fault]
    rng = np.random.default_rng(seed)
    raw, estimates, reference, imu = [], [], [], []
    for time in np.round(np.arange(0, 10.01, 0.1), 3):
        target = truth(time)
        damaged = start <= time < end
        rgb_bad = damaged and fault == "rgb_low"
        depth_bad = damaged and fault != "rgb_low"
        valid = not (damaged and fault == "rgb_depth_outage")
        depth = 0.08 if depth_bad else 0.94
        rgb = 0.18 if rgb_bad else (0.88 if valid else 0)
        inliers = int(rng.integers(78, 105)) if valid else 0
        if rgb_bad:
            inliers = int(rng.integers(8, 20))
        position = target.position + rng.normal(0, 0.007, 3)
        rotation = target.rotation * Rotation.from_rotvec(rng.normal(0, 0.004, 3))
        if rgb_bad:
            position += [0.8, 0.1, 0]
        if damaged and fault == "depth_loss":
            position += [0.04, -0.03, 0.01]
        raw.append(dict(timestamp=time, valid=int(valid), state="OK" if valid else "LOST",
                        quality_rgb=rgb, quality_depth=depth, source="slam" if valid else "none",
                        latency_ms="", rgb_inliers=inliers))
        estimates.append((time, Pose(position, rotation) if valid else None))
        reference.append((time, target))

    for time in np.round(np.arange(0, 10.001, 0.01), 3):
        acceleration = np.array([-0.22 * np.sin(time), -0.32 * 0.7**2 * np.cos(0.7 * time), 0])
        specific_force = truth(time).rotation.inv().apply(acceleration + [0, 0, 9.81])
        gyro = np.array([0, 0, 0.18 + 0.12 * 0.8 * np.cos(0.8 * time)])
        if fault == "rgb_depth_outage" and seed == 2 and start <= time < end:
            specific_force += [0.55, 0, 0]
        gyro += rng.normal(0, 0.001, 3)
        specific_force += rng.normal(0, 0.014, 3)
        imu.append(dict(zip(("timestamp", "gx", "gy", "gz", "ax", "ay", "az"),
                            [time, *gyro, *specific_force])))
    write_csv(folder / "tracking.csv", raw)
    write_csv(folder / "imu.csv", imu)
    write_csv(folder / "faults.csv", [dict(fault=fault, start_s=start, end_s=end)])
    write_tum(folder / "trajectory_est.tum", estimates)
    write_tum(folder / "trajectory_gt.tum", reference)
