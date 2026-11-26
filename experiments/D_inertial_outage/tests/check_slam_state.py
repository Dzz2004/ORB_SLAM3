"""检查状态读取的关键边界；数据仅用于测试，不作为SLAM实验结果。"""

import tempfile
from pathlib import Path
import unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'integration'))
import pandas as pd

from slam_state import FIELDS, read_states, last_trusted_state


def sample(t, valid=1, map_id=0):
    row = dict.fromkeys(FIELDS, 0.0)
    row.update(timestamp=t, qw=1, valid=valid, imu_initialized=1, map_id=map_id)
    return row


class StateChecks(unittest.TestCase):
    def test_不读取中断后的状态(self):
        frame = pd.DataFrame([sample(0.9), sample(1.0, 0), sample(1.1)])
        state, history = last_trusted_state(frame, 1)
        self.assertEqual(state.timestamp, 0.9)
        self.assertTrue((history.timestamp <= 1).all())

    def test_拒绝旧地图状态(self):
        frame = pd.DataFrame([sample(0.9), sample(1, 0, 1)])
        with self.assertRaises(ValueError):
            last_trusted_state(frame, 1)

    def test_拒绝过期状态(self):
        with self.assertRaises(ValueError):
            last_trusted_state(pd.DataFrame([sample(0.5)]), 1)

    def test_不跨世界坐标版本取状态(self):
        frame = pd.DataFrame([dict(sample(0.9), frame_epoch=0),
                              dict(sample(1, 0), frame_epoch=1)])
        with self.assertRaises(ValueError):
            last_trusted_state(frame, 1)

    def test_完整读取并保留偏置(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / '测试.csv'
            row = sample(0.9)
            row.update(vx=0.3, bgx=0.01, bax=0.2)
            pd.DataFrame([row, sample(1.0)]).to_csv(path, index=False)
            frame = read_states(path)
            self.assertEqual(frame.iloc[0].bgx, 0.01)
            self.assertEqual(frame.iloc[0].bax, 0.2)
            self.assertEqual(frame.iloc[0].vx, 0.3)

    def test_缺少速度时不使用真值补齐(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / '测试.csv'
            pd.DataFrame([sample(1)]).drop(columns='vx').to_csv(path, index=False)
            with self.assertRaises(ValueError):
                read_states(path)


if __name__ == '__main__':
    unittest.main(testRunner=unittest.TextTestRunner(stream=sys.stdout, verbosity=2))
