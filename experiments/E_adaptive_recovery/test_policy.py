import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from degradation_policy import DegradationPolicy, ImuPredictor, Pose, State
from replay import evaluate, read_case, replay
from run import CONFIG
from synthetic import CASES, SEEDS, make_case


CONFIG_VALUES = json.loads(CONFIG.read_text(encoding="utf-8"))


def pose(x):
    return Pose(np.array([float(x), 0, 0]), Rotation.identity())


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.config = CONFIG_VALUES.copy()

    def test_all_states_and_hold(self):
        policy = DegradationPolicy(self.config)
        self.assertEqual(policy.step(0, pose(0), 1, 1).state, State.NORMAL)
        self.assertEqual(policy.step(0.1, pose(0), 1, 0).state, State.DEPTH_DEGRADED)
        self.assertEqual(policy.step(0.2, pose(0), 1, 1).state, State.DEPTH_DEGRADED)
        self.assertEqual(policy.step(0.3, pose(0), 1, 1).state, State.NORMAL)
        self.assertEqual(policy.step(0.4, None, 0, 1).state, State.VISION_DEGRADED)
        self.assertEqual(policy.step(0.5, None, 0, 0).state, State.IMU_ONLY)
        self.assertEqual(policy.step(0.6, pose(0), 1, 1).state, State.IMU_ONLY)
        self.assertEqual(policy.step(0.7, pose(0), 1, 1).state, State.RECOVERY)
        self.assertIsNone(policy.step(0.8, pose(0), 1, 1).pose)
        self.assertEqual(policy.step(0.9, pose(0), 1, 1).state, State.NORMAL)

    def test_recovery_streak_resets_after_pose_jump_and_frame_gap(self):
        policy = DegradationPolicy(self.config)
        policy.step(0, None, 0, 1)
        policy.step(0.2, pose(0), 1, 1)
        policy.step(0.3, pose(0), 1, 1)
        self.assertIsNone(policy.step(0.4, pose(5), 1, 1).pose)
        self.assertIsNone(policy.step(0.5, pose(5), 1, 1).pose)
        self.assertIsNone(policy.step(1.0, pose(5), 1, 1).pose)
        self.assertIsNone(policy.step(1.1, pose(5), 1, 1).pose)
        self.assertEqual(policy.step(1.2, pose(5), 1, 1).state, State.NORMAL)

    def test_prediction_timeout_and_missing_imu(self):
        times = np.arange(0, 3.01, 0.01)
        samples = np.column_stack((times, np.zeros((len(times), 5)), np.full(len(times), 9.81)))
        predictor = ImuPredictor(self.config, samples)
        predictor.accept(0, pose(0))
        self.assertIsNone(predictor.predict(0.1)[0])
        predictor.accept(0.1, pose(0.1))
        prediction, age = predictor.predict(2.1)
        self.assertAlmostEqual(age, 2.0)
        np.testing.assert_allclose(prediction.position, [2.1, 0, 0], atol=1e-8)
        self.assertIsNone(predictor.predict(2.10001)[0])
        predictor.samples = samples[(times < 0.3) | (times > 0.4)]
        self.assertIsNone(predictor.predict(0.5)[0])

    def test_missing_depth_is_unknown(self):
        policy = DegradationPolicy(self.config)
        self.assertEqual(policy.step(0, None, 0, None).state, State.VISION_DEGRADED)

    def test_matrix_and_gt_independence(self):
        states = set()
        with tempfile.TemporaryDirectory() as directory:
            for fault in CASES:
                for seed in SEEDS:
                    folder = Path(directory) / fault / str(seed)
                    make_case(folder, fault, seed)
                    frames, imu, faults = read_case(folder, self.config)
                    outcome = replay(frames, imu, "monitor_imu", self.config)
                    changed_gt = [replace(frame, reference=None) for frame in frames]
                    outcome_no_gt = replay(changed_gt, imu, "monitor_imu", self.config)
                    for (first, _, _), (second, _, _) in zip(outcome, outcome_no_gt):
                        states.add(first.state)
                        self.assertEqual((first.state, first.source), (second.state, second.source))
                        if first.pose is not None:
                            np.testing.assert_array_equal(first.pose.position, second.pose.position)
                            np.testing.assert_array_equal(first.pose.rotation.as_quat(), second.pose.rotation.as_quat())
                    metrics, events, _ = evaluate(frames, outcome, faults, self.config)
                    self.assertEqual(len(events), len(frames))
                    if fault == "rgb_depth_outage":
                        self.assertGreater(metrics["prediction_coverage_s"], 0)
                        self.assertLessEqual(max(row["prediction_age_s"] or 0 for row in events), 2 + 1e-9)
                    elif fault == "rgb_low":
                        self.assertTrue(all(row["state"] == "RECOVERY" for row in events if row["source"] == "imu"))
                        self.assertGreater(metrics["wrong_takeover_frames"], 0)
                    elif fault == "depth_loss":
                        self.assertEqual(metrics["predicted_frames"], 0)
        self.assertEqual(states, set(State))


if __name__ == "__main__":
    unittest.main()
