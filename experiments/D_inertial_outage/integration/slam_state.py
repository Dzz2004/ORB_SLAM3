"""严格读取实际SLAM初始状态，不从真值补齐速度或偏置。"""

import numpy as np
import pandas as pd

FIELDS = ['timestamp', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw',
          'vx', 'vy', 'vz', 'bgx', 'bgy', 'bgz', 'bax', 'bay', 'baz',
          'valid', 'imu_initialized', 'map_id']


def read_states(path):
    """约定：秒、米、弧度，IMU机体到SLAM世界的四元数，偏置在IMU机体系。"""
    frame = pd.read_csv(path)
    missing = set(FIELDS) - set(frame.columns)
    if missing:
        raise ValueError('SLAM状态文件缺少列：' + ','.join(sorted(missing)))
    # B/C已确认同一地图也可能因IMU初始化或回环改变世界坐标，需同时检查坐标版本。
    if 'frame_epoch' not in frame:
        frame['frame_epoch'] = 0
    frame = frame[FIELDS + ['frame_epoch']].apply(pd.to_numeric, errors='raise')
    if not np.isfinite(frame.to_numpy()).all():
        raise ValueError('SLAM状态包含缺失或无穷数值')
    if frame.empty or (np.diff(frame.timestamp) <= 0).any():
        raise ValueError('状态不能为空，时间戳须严格递增；重启后的日志应另存文件')
    if not frame.valid.isin([0, 1]).all() or not frame.imu_initialized.isin([0, 1]).all():
        raise ValueError('有效状态和IMU初始化标记只能为0或1')
    norm = np.linalg.norm(frame[['qx', 'qy', 'qz', 'qw']], axis=1)
    usable = (frame.valid == 1) & (frame.imu_initialized == 1)
    if not np.allclose(norm[usable], 1, atol=1e-3):
        raise ValueError('有效状态的四元数必须归一化')
    return frame


def last_trusted_state(frame, outage_time, max_age=0.15):
    """仅选择中断前已经得到的状态，禁止读取未来帧或恢复后的估计。"""
    past = frame[frame.timestamp <= outage_time + 1e-10]
    if past.empty:
        raise ValueError('中断之前没有SLAM状态')
    # 只在当前地图里选取，不能在地图重置后回头取旧地图中的状态。
    past = past[past.map_id == past.iloc[-1].map_id]
    if 'frame_epoch' in past:
        past = past[past.frame_epoch == past.iloc[-1].frame_epoch]
    valid = past[(past.valid == 1) & (past.imu_initialized == 1)]
    if valid.empty:
        raise ValueError('中断之前没有完成IMU初始化的可信状态')
    state = valid.iloc[-1]
    if outage_time - state.timestamp > max_age:
        raise ValueError('最后可信状态距离中断太久，拒绝用过期状态初始化')
    return state, valid
