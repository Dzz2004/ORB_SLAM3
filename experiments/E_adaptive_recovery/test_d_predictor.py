import json
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from d_predictor import DImuPredictor
from degradation_policy import Pose
from run import CONFIG


class DAdapterTests(unittest.TestCase):
    def test_causal_prediction_gap_and_timeout(self):
        config = json.loads(CONFIG.read_text(encoding="utf-8"))
        times = np.arange(302) / 100
        samples = np.column_stack((times, np.zeros((len(times), 5)), np.full(len(times), 9.81)))
        predictor = DImuPredictor(config, samples)
        for time in (0.0, 0.101):
            predictor.accept(time, Pose(np.array([time, 0, 0]), Rotation.identity()))
        target = 0.503
        pose, age = predictor.predict(target)
        self.assertAlmostEqual(age, target - 0.101)
        np.testing.assert_allclose(pose.position, [target, 0, 0], atol=1e-11)
        samples[times > target, 1:] = 100
        repeated, _ = predictor.predict(target)
        np.testing.assert_array_equal(pose.position, repeated.position)
        self.assertIsNone(predictor.predict(2.102)[0])
        predictor.samples = samples[(times < 0.3) | (times > 0.4)]
        self.assertIsNone(predictor.predict(target)[0])
