"""使用答案已知的简单运动检查计算是否正确；这些不是数据集实验结果。"""

import unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation
from imu_dead_reckoning import predict_imu


class IntegratorChecks(unittest.TestCase):
    def setUp(self):
        # 构造两秒、200赫兹的数据；静止时比力向上抵消重力。
        self.t = np.linspace(0, 2, 401)
        self.zero = np.zeros((len(self.t), 3))
        self.rest_force = np.tile([0, 0, 9.81], (len(self.t), 1))

    def run_prediction(self, gyro=None, accel=None, **kwargs):
        return predict_imu(self.t, self.zero if gyro is None else gyro,
                           self.rest_force if accel is None else accel,
                           kwargs.pop('position', [0, 0, 0]),
                           kwargs.pop('quaternion', [0, 0, 0, 1]),
                           kwargs.pop('velocity', [0, 0, 0]), **kwargs)

    def test_stationary(self):
        """检查一：设备静止时，预测位置应保持不变。"""
        result = self.run_prediction()
        np.testing.assert_allclose(result['positions'], 0, atol=1e-10)

    def test_constant_velocity(self):
        """检查二：每秒前进一米，两秒后应前进两米。"""
        result = self.run_prediction(velocity=[1, 0, 0])
        np.testing.assert_allclose(result['positions'][-1], [2, 0, 0], atol=1e-10)

    def test_constant_rotation(self):
        """检查三：每秒绕Z轴旋转45度，两秒后应旋转90度。"""
        w = np.tile([0, 0, np.pi / 4], (len(self.t), 1))
        result = self.run_prediction(gyro=w)
        expected = Rotation.from_euler('z', 90, degrees=True)
        error = (expected.inv() * Rotation.from_quat(result['quaternions'][-1])).magnitude()
        self.assertLess(error, 1e-10)
        np.testing.assert_allclose(result['positions'], 0, atol=1e-10)

    def test_tilted_body_acceleration_and_bias(self):
        """检查四：设备倾斜且测量带偏置时，仍应算对已知加速度的运动。"""
        # 这个检查可以发现旋转方向弄反、重力补偿或偏置处理错误。
        r = Rotation.from_euler('xyz', [25, -35, 40], degrees=True)
        world_accel = np.array([0.8, -0.3, 0.2])
        gravity = np.array([0, 0, -9.81])
        ba, bg = np.array([0.03, -0.02, 0.01]), np.array([0.001, 0.002, -0.003])
        f = np.tile(r.inv().apply(world_accel - gravity) + ba, (len(self.t), 1))
        result = self.run_prediction(gyro=np.tile(bg, (len(self.t), 1)), accel=f,
                                     quaternion=r.as_quat(), gyro_bias=bg, accel_bias=ba)
        np.testing.assert_allclose(result['positions'][-1], 2 * world_accel, atol=1e-10)
        np.testing.assert_allclose(result['velocities'][-1], 2 * world_accel, atol=1e-10)

    def test_reject_repeated_timestamp(self):
        """检查五：时间戳重复时应明确报错。"""
        self.t[1] = self.t[0]
        with self.assertRaises(ValueError):
            self.run_prediction()


if __name__ == '__main__':
    unittest.main(testRunner=unittest.TextTestRunner(stream=sys.stdout, verbosity=2))
