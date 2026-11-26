"""Causal degradation and recovery using pose estimates and IMU only."""

from dataclasses import dataclass
from enum import Enum

import numpy as np
from scipy.spatial.transform import Rotation


class State(str, Enum):
    NORMAL = "NORMAL"
    VISION_DEGRADED = "VISION_DEGRADED"
    DEPTH_DEGRADED = "DEPTH_DEGRADED"
    IMU_ONLY = "IMU_ONLY"
    RECOVERY = "RECOVERY"


@dataclass
class Pose:
    position: np.ndarray
    rotation: Rotation  # world-from-IMU, or a co-located and aligned camera


@dataclass
class Decision:
    state: State
    source: str
    pose: Pose | None
    prediction_age_s: float | None = None


def pose_error(first, second):
    return (float(np.linalg.norm(first.position - second.position)),
            float(np.degrees((first.rotation.inv() * second.rotation).magnitude())))


class ImuPredictor:
    """Bounded strapdown prediction. Initial velocity comes from two accepted poses."""

    def __init__(self, config, samples):
        self.config = config
        self.samples = samples
        self.anchor = None
        self.previous = None
        self.velocity = None

    def accept(self, timestamp, pose):
        self.velocity = None
        if self.previous is not None:
            last_time, last_pose = self.previous
            dt = timestamp - last_time
            if 0 < dt <= self.config["max_frame_gap_s"]:
                self.velocity = (pose.position - last_pose.position) / dt
        self.anchor = self.previous = (timestamp, pose)

    def break_velocity_history(self):
        self.previous = None

    def predict(self, timestamp):
        if self.anchor is None or self.velocity is None or len(self.samples) == 0:
            return None, None
        start, pose = self.anchor
        age = timestamp - start
        if not 0 < age <= self.config["prediction_max_s"] + 1e-9:
            return None, None
        times = self.samples[:, 0]
        index = np.searchsorted(times, start, side="right") - 1
        if index < 0:
            return None, None
        position, rotation = pose.position.copy(), pose.rotation
        velocity = self.velocity.copy()
        current = start
        while current < timestamp - 1e-10:
            next_time = times[index + 1] if index + 1 < len(times) else np.inf
            end = min(timestamp, next_time)
            if end - times[index] > self.config["max_imu_gap_s"] + 1e-9:
                return None, None
            dt = end - current
            gyro = self.samples[index, 1:4] - self.config["gyro_bias_radps"]
            accel = self.samples[index, 4:7] - self.config["accel_bias_mps2"]
            midpoint = rotation * Rotation.from_rotvec(gyro * (dt / 2))
            acceleration = midpoint.apply(accel) + self.config["gravity_mps2"]
            position += velocity * dt + 0.5 * acceleration * dt * dt
            velocity += acceleration * dt
            rotation = rotation * Rotation.from_rotvec(gyro * dt)
            current = end
            index += 1
        return Pose(position, rotation), age


class DegradationPolicy:
    def __init__(self, config, predictor=None):
        self.config = config
        self.predictor = predictor
        self.state = None
        self.entered = None
        self.previous_time = None
        self.reliable_count = 0
        self.recovery_candidate = None
        self.needs_recovery = False

    def _enter(self, state, timestamp):
        if state != self.state:
            self.state = state
            self.entered = timestamp

    def step(self, timestamp, slam, rgb, depth):
        rgb_ok = slam is not None and rgb >= self.config["rgb_min"]
        depth_bad = depth is not None and depth < self.config["depth_min"]
        if self.previous_time is not None:
            if timestamp - self.previous_time > self.config["max_frame_gap_s"] + 1e-9:
                self.reliable_count = 0
                self.recovery_candidate = None
                self.needs_recovery = True
                if self.predictor:
                    self.predictor.break_velocity_history()
        self.previous_time = timestamp

        if not rgb_ok:
            self.needs_recovery = True
            self.reliable_count = 0
            self.recovery_candidate = None
            if self.predictor:
                self.predictor.break_velocity_history()
            self._enter(State.IMU_ONLY if depth_bad else State.VISION_DEGRADED, timestamp)
        elif self.needs_recovery:
            hold_met = timestamp - self.entered + 1e-9 >= self.config["min_state_hold_s"]
            if self.state == State.RECOVERY or hold_met:
                self._enter(State.RECOVERY, timestamp)
                prediction, _ = self.predictor.predict(timestamp) if self.predictor else (None, None)
                reference = prediction if prediction is not None else self.recovery_candidate
                self.recovery_candidate = slam
                consistent = reference is None or self._consistent(reference, slam)
                self.reliable_count = self.reliable_count + 1 if consistent else 0
                if (self.reliable_count >= self.config["recovery_frames"]
                        and timestamp - self.entered + 1e-9 >= self.config["min_state_hold_s"]):
                    self.needs_recovery = False

        if rgb_ok and not self.needs_recovery:
            target = State.DEPTH_DEGRADED if depth_bad else State.NORMAL
            if self.state == State.DEPTH_DEGRADED and target == State.NORMAL:
                if timestamp - self.entered + 1e-9 < self.config["min_state_hold_s"]:
                    target = State.DEPTH_DEGRADED
            self._enter(target, timestamp)
            if self.predictor:
                self.predictor.accept(timestamp, slam)
            return Decision(self.state, "slam", slam)

        if self.predictor and self.state in (State.IMU_ONLY, State.RECOVERY):
            pose, age = self.predictor.predict(timestamp)
            if pose is not None:
                return Decision(self.state, "imu", pose, age)
        return Decision(self.state, "none", None)

    def _consistent(self, reference, slam):
        position, angle = pose_error(reference, slam)
        return (position <= self.config["recovery_position_gate_m"]
                and angle <= self.config["recovery_rotation_gate_deg"])
