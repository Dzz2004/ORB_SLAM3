"""Read the group exchange formats and replay the three policy variants."""

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from degradation_policy import Decision, DegradationPolicy, ImuPredictor, Pose, State, pose_error
from quality_monitor import QualityMonitor


VARIANTS = ("baseline", "monitor", "monitor_imu")


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows, fields=None):
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields or list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def check_times(values, name):
    if not all(map(math.isfinite, values)) or any(a >= b for a, b in zip(values, values[1:])):
        raise ValueError(f"{name}: timestamps must be finite and strictly increasing")


class Trajectory:
    def __init__(self, path):
        data = np.loadtxt(path, ndmin=2)
        self.times = np.array([])
        self.poses = []
        if data.size == 0:
            return
        if data.shape[1] != 8 or not np.isfinite(data).all():
            raise ValueError(f"{path}: expected 8 finite TUM fields")
        check_times(data[:, 0], path)
        self.times = data[:, 0]
        self.poses = [Pose(row[1:4], Rotation.from_quat(row[4:8])) for row in data]

    def at(self, timestamp, tolerance):
        index = np.searchsorted(self.times, timestamp)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(self.times)]
        if not candidates:
            return None
        nearest = min(candidates, key=lambda i: abs(self.times[i] - timestamp))
        return self.poses[nearest] if abs(self.times[nearest] - timestamp) <= tolerance else None


@dataclass
class Frame:
    time: float
    slam: Pose | None
    reference: Pose | None
    raw_state: str
    rgb: float | None
    depth: float | None
    inliers: int | None
    latency_ms: float | None


def optional_float(row, field, maximum=None):
    value = row.get(field, "").strip()
    if not value:
        return None
    value = float(value)
    if not math.isfinite(value) or value < 0 or (maximum is not None and value > maximum):
        raise ValueError(f"{field}: out-of-range value {value}")
    return value


def read_case(folder, config):
    folder = Path(folder)
    estimate = Trajectory(folder / "trajectory_est.tum")
    ground_truth = Trajectory(folder / "trajectory_gt.tum")
    tolerance = config["timestamp_tolerance_s"]
    frames = []
    for row in read_csv(folder / "tracking.csv"):
        timestamp = float(row["timestamp"])
        valid = row["valid"].strip().lower()
        if valid not in {"1", "0", "true", "false"}:
            raise ValueError(f"Invalid tracking.valid value {valid!r}")
        state = row["state"].strip().upper()
        source = row["source"].strip().lower()
        eligible = (valid in {"1", "true"} and source == "slam"
                    and state not in {"LOST", "RECENTLY_LOST", "NOT_INITIALIZED",
                                      "NO_IMAGES_YET", "SYSTEM_NOT_READY", "-1", "0", "1", "3", "4"})
        pose = estimate.at(timestamp, tolerance) if eligible else None
        count = optional_float(row, "rgb_inliers")
        if count is not None and not count.is_integer():
            raise ValueError("rgb_inliers must be an integer count")
        frames.append(Frame(timestamp, pose, ground_truth.at(timestamp, tolerance), state,
                            optional_float(row, "quality_rgb", 1), optional_float(row, "quality_depth", 1),
                            int(count) if count is not None else None, optional_float(row, "latency_ms")))
    if len(frames) < 2:
        raise ValueError("tracking.csv requires at least two frames")
    check_times([frame.time for frame in frames], "tracking.csv")

    imu_path = folder / "imu.csv"
    imu = np.empty((0, 7))
    if imu_path.exists():
        fields = ("timestamp", "gx", "gy", "gz", "ax", "ay", "az")
        imu = np.array([[float(row[key]) for key in fields] for row in read_csv(imu_path)]).reshape(-1, 7)
        if not np.isfinite(imu).all():
            raise ValueError("imu.csv contains non-finite values")
        check_times(imu[:, 0], "imu.csv")

    faults = []
    for row in read_csv(folder / "faults.csv"):
        start, end = float(row["start_s"]), float(row["end_s"])
        if not (math.isfinite(start) and math.isfinite(end) and frames[0].time <= start < end <= frames[-1].time):
            raise ValueError("Fault windows must lie within the recording and have positive duration")
        if faults and start < faults[-1][2]:
            raise ValueError("Fault windows must be ordered and nonoverlapping")
        faults.append((row["fault"], start, end))
    return frames, imu, faults


def replay(frames, samples, variant, config):
    monitor = QualityMonitor(config)
    policy = DegradationPolicy(config, ImuPredictor(config, samples) if variant == "monitor_imu" else None)
    decisions = []
    for frame in frames:
        rgb, depth = monitor.update(frame.time, frame.slam is not None, frame.rgb, frame.depth, frame.inliers)
        if variant == "baseline":
            decision = (Decision(State.NORMAL, "slam", frame.slam) if frame.slam is not None
                        else Decision(State.VISION_DEGRADED, "none", None))
        else:
            decision = policy.step(frame.time, frame.slam, rgb, depth)
        decisions.append((decision, rgb, depth))
    return decisions


def within_limits(estimate, reference, config):
    if estimate is None or reference is None:
        return False
    position, angle = pose_error(estimate, reference)
    return position <= config["evaluation_position_m"] and angle <= config["evaluation_rotation_deg"]


def relative_pose(first, second):
    inverse = first.rotation.inv()
    return Pose(inverse.apply(second.position - first.position), inverse * second.rotation)


def rmse(errors):
    return float(np.sqrt(np.mean(np.square(errors)))) if errors else None


def evaluate(frames, replayed, faults, config):
    decisions = [value[0] for value in replayed]
    times = np.array([frame.time for frame in frames])
    step = float(np.median(np.diff(times)))
    durations = np.r_[np.minimum(np.diff(times), step), step]
    errors, relative_errors, events, recovery = [], [], [], []
    normal_total = normal_correct = wrong_takeovers = evaluable_takeovers = 0
    previous_state = None
    previous_pair = None
    loss_events = 0
    was_lost = False
    for index, (frame, (decision, rgb, depth)) in enumerate(zip(frames, replayed)):
        active = next((label for label, start, end in faults if start <= frame.time < end), "")
        correct = within_limits(decision.pose, frame.reference, config)
        if not active and frame.reference is not None:
            normal_total += 1
            normal_correct += correct
        if decision.source == "imu" and frame.reference is not None:
            evaluable_takeovers += 1
            wrong_takeovers += within_limits(frame.slam, frame.reference, config)
        paired = decision.pose is not None and frame.reference is not None
        error = pose_error(decision.pose, frame.reference) if paired else (None, None)
        if paired:
            errors.append(error[0])
            if previous_pair is not None and times[index] - times[index - 1] <= config["max_frame_gap_s"]:
                old_pose, old_ref = previous_pair
                relative_errors.append(pose_error(relative_pose(old_pose, decision.pose),
                                                  relative_pose(old_ref, frame.reference)))
        previous_pair = (decision.pose, frame.reference) if paired else None
        lost = frame.slam is None
        loss_events += lost and not was_lost
        was_lost = lost
        events.append(dict(timestamp=frame.time, fault=active, state=decision.state.value,
                           transition=decision.state.value if decision.state != previous_state else "",
                           valid=int(decision.pose is not None), source=decision.source,
                           raw_state=frame.raw_state, raw_slam_valid=int(not lost),
                           quality_rgb=rgb, quality_depth=depth, rgb_inliers=frame.inliers,
                           latency_ms=frame.latency_ms, prediction_age_s=decision.prediction_age_s,
                           position_error_m=error[0], rotation_error_deg=error[1],
                           within_error_limit=int(correct) if frame.reference is not None else None))
        previous_state = decision.state

    for fault_index, (label, start, end) in enumerate(faults):
        cutoff = faults[fault_index + 1][1] if fault_index + 1 < len(faults) else math.inf
        streak = 0
        recovered = None
        previous = None
        for frame, decision in zip(frames, decisions):
            if not end <= frame.time < cutoff:
                continue
            if previous is not None and frame.time - previous > config["max_frame_gap_s"]:
                streak = 0
            previous = frame.time
            reliable = decision.source == "slam" and within_limits(decision.pose, frame.reference, config)
            streak = streak + 1 if reliable else 0
            if streak >= config["recovery_frames"]:
                recovered = frame.time - end
                break
        predicted_in_fault = sum(max(0.0, min(t + dt, end) - max(t, start))
                                 for t, dt, d in zip(times, durations, decisions) if d.source == "imu")
        recovery.append(dict(fault=label, start_s=start, end_s=end, recovery_s=recovered,
                             recovered=int(recovered is not None), prediction_in_fault_s=predicted_in_fault))
    metrics = dict(
        frame_count=len(frames), gt_paired_frames=len(errors), rpe_pair_count=len(relative_errors),
        output_coverage=sum(d.pose is not None for d in decisions) / len(frames),
        slam_coverage=sum(d.source == "slam" for d in decisions) / len(frames),
        raw_slam_loss_rate=sum(f.slam is None for f in frames) / len(frames),
        raw_slam_loss_events=loss_events,
        normal_frame_accuracy=normal_correct / normal_total if normal_total else None,
        normal_evaluable_frames=normal_total,
        wrong_takeover_frames=wrong_takeovers if evaluable_takeovers else (None if any(d.source == "imu" for d in decisions) else 0),
        evaluable_takeover_frames=evaluable_takeovers,
        predicted_frames=sum(d.source == "imu" for d in decisions),
        prediction_coverage_s=float(sum(dt for dt, d in zip(durations, decisions) if d.source == "imu")),
        ate_rmse_m=rmse(errors), rpe_translation_rmse_m=rmse([e[0] for e in relative_errors]),
        rpe_rotation_rmse_deg=rmse([e[1] for e in relative_errors]),
        recovered_events=sum(row["recovered"] for row in recovery), fault_events=len(faults),
    )
    return metrics, events, recovery


def write_tum(path, samples):
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("# timestamp tx ty tz qx qy qz qw\n")
        for timestamp, pose in samples:
            if pose is not None:
                values = np.r_[timestamp, pose.position, pose.rotation.as_quat()]
                handle.write(" ".join(f"{value:.12f}" for value in values) + "\n")
