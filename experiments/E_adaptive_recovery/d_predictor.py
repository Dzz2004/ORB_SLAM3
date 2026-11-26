"""Apply E's timeout and sample checks before calling D's inertial integrator."""

import numpy as np
from scipy.spatial.transform import Rotation

from degradation_policy import ImuPredictor, Pose
from upstream_d.imu_dead_reckoning import predict_imu


class DImuPredictor(ImuPredictor):
    def predict(self, timestamp):
        if self.anchor is None or self.velocity is None or len(self.samples) == 0:
            return None, None
        start, pose = self.anchor
        age = timestamp - start
        if not 0 < age <= self.config["prediction_max_s"] + 1e-9:
            return None, None

        times = self.samples[:, 0]
        first = np.searchsorted(times, start, side="right") - 1
        end = np.searchsorted(times, timestamp, side="right")
        if first < 0:
            return None, None
        available = self.samples[first:end]
        with_boundaries = np.r_[available[:, 0], timestamp]
        if np.any(np.diff(with_boundaries) > self.config["max_imu_gap_s"] + 1e-9):
            return None, None

        interior = available[1:][available[1:, 0] < timestamp]
        # Hold the most recent measurement at each endpoint. No future sample is read.
        window = np.vstack((available[0], interior, available[-1]))
        window[0, 0] = start
        window[-1, 0] = timestamp
        result = predict_imu(window[:, 0], window[:, 1:4], window[:, 4:7],
                             pose.position, pose.rotation.as_quat(), self.velocity,
                             self.config["gyro_bias_radps"], self.config["accel_bias_mps2"],
                             self.config["gravity_mps2"])
        return Pose(result["positions"][-1], Rotation.from_quat(result["quaternions"][-1])), age
