"""仅使用IMU预测位置与朝向；单位使用米、秒、弧度，四元数按x、y、z、w排列。"""

import numpy as np
from scipy.spatial.transform import Rotation


def predict_imu(timestamps, gyro, accel, position, quaternion, velocity,
                gyro_bias=None, accel_bias=None, gravity=(0.0, 0.0, -9.81)):
    """根据相邻两条IMU测量的平均值，逐步推算设备的位置、速度和朝向。

    timestamps：时间戳，单位为秒，第一项必须对应初始状态的时刻。
    gyro：设备自身坐标系中的三轴角速度，单位为弧度/秒。
    accel：设备自身坐标系中的三轴比力，单位为米/秒²。
    position、velocity：世界坐标系中的初始位置和初始速度。
    quaternion：初始朝向，将设备坐标系中的向量旋转到世界坐标系。
    gyro_bias、accel_bias：需要从测量中减去的陀螺仪和加速度计偏置。
    gravity：世界坐标系的重力向量；默认世界Z轴朝上。

    本函数要求输入加速度为“比力”：旋转到世界坐标系后加上重力，才得到运动加速度。
    数据集若使用不同定义，需要先转换。函数内部不会读取真实轨迹或用真值纠偏。
    返回所有输入时刻的位置、速度、姿态，包括初始时刻。
    """
    # 先统一输入格式，防止数据长度不一致或时间戳倒序。
    t = np.asarray(timestamps, dtype=float)
    w = np.asarray(gyro, dtype=float)
    f = np.asarray(accel, dtype=float)
    if t.ndim != 1 or len(t) < 2 or w.shape != (len(t), 3) or f.shape != w.shape:
        raise ValueError('至少需要两条时间戳，角速度和加速度均应为N行3列')
    if not all(np.isfinite(x).all() for x in (t, w, f)) or np.any(np.diff(t) <= 0):
        raise ValueError('数据不能包含无穷或缺失值，时间戳必须严格递增')

    def vec3(value):
        value = np.asarray(value, dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError('位置、速度、偏置和重力必须各包含三个有限数值')
        return value.copy()

    p, v, g = vec3(position), vec3(velocity), vec3(gravity)
    bg = vec3(np.zeros(3) if gyro_bias is None else gyro_bias)
    ba = vec3(np.zeros(3) if accel_bias is None else accel_bias)
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) == 0:
        raise ValueError('四元数必须包含四个有限数值，且不能全部为零')
    r = Rotation.from_quat(q)
    positions, velocities, quaternions = [p.copy()], [v.copy()], [r.as_quat()]
    for k, dt in enumerate(np.diff(t)):
        # 第一步：取相邻角速度的平均值，减去偏置，得到这段时间内的旋转。
        omega = 0.5 * (w[k] + w[k + 1]) - bg
        r_next = r * Rotation.from_rotvec(omega * dt)
        # 第二步：分别将两端的比力转到世界坐标系，取平均并加回重力。
        a_world = 0.5 * (r.apply(f[k] - ba) + r_next.apply(f[k + 1] - ba)) + g
        # 第三步：根据加速度更新位置和速度，再记录结果。
        p = p + v * dt + 0.5 * a_world * dt**2
        v = v + a_world * dt
        r = r_next
        positions.append(p.copy())
        velocities.append(v.copy())
        quaternions.append(r.as_quat())
    return {'timestamps': t.copy(), 'positions': np.array(positions),
            'velocities': np.array(velocities), 'quaternions': np.array(quaternions)}
