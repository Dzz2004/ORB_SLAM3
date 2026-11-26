"""按D分工运行IMU短时预测，分别支持理想初始化与实际SLAM初始化。"""

import argparse
import hashlib
import io
import json
import os
import sys
import shutil
import subprocess
from time import perf_counter
from pathlib import Path
import zipfile

# 绘图缓存放在仓库外的数据目录，避免实验文件夹出现临时字体文件。
os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).resolve().parents[3] / 'data_D/cache/matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from imu_dead_reckoning import predict_imu

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA = ROOT.parent / 'data_D/downloads/ArchVizTinyHouseDay_easy_imu.zip'
EXPECTED_HASH = 'cf34be5926c7fa24006322654586d925913682aac879783d789e270b4771058c'
HASHES = {'easy': EXPECTED_HASH,
          'hard': 'd6b8bc740ea1a97204f85f890562dbb2717de3c0ed0b49faaf16e8af6dcee801'}


def load_sequence(archive, sequence, difficulty='easy'):
    """从压缩包直接读取，不解压大批重复文件；严格核对数据约定。"""
    if hashlib.sha256(archive.read_bytes()).hexdigest() != HASHES[difficulty]:
        raise ValueError('数据包校验失败，必须使用下载脚本指定难度的官方惯性包')
    prefix = f'ArchVizTinyHouseDay/Data_{difficulty}/{sequence}/imu/'
    with zipfile.ZipFile(archive) as package:
        names = ['imu_time', 'pos_global', 'vel_global', 'ori_global',
                 'gyro', 'acc', 'acc_nograv', 'acc_nograv_body']
        data = {name: np.load(io.BytesIO(package.read(prefix + name + '.npy')),
                              allow_pickle=False) for name in names}
    t = data['imu_time']
    if t.ndim != 1 or not np.all(np.diff(t) > 0):
        raise ValueError('数据时间轴不合法')
    for name, array in data.items():
        shape = (len(t),) if name == 'imu_time' else (len(t), 3)
        if array.shape != shape or not np.isfinite(array).all():
            raise ValueError(f'{name}的形状或数值不合法')

    # 官方包中的朝向是三列欧拉角，不是四元数。
    # 大写XYZ表示依次绕自身坐标轴旋转。下面使用冗余真值字段验证，不能凭猜测选顺序。
    r = Rotation.from_euler('XYZ', data['ori_global'])
    residual = np.max(np.linalg.norm(r.apply(data['acc_nograv_body']) -
                                    data['acc_nograv'], axis=1))
    # 该包采用Z轴向下的世界坐标；比力转到世界后加[0,0,9.8]才得到运动加速度。
    gravity_check = r.apply(data['acc'] - data['acc_nograv_body'])
    if residual > 1e-7 or not np.allclose(gravity_check, [0, 0, -9.8], atol=1e-7):
        raise ValueError('坐标或重力约定验证失败，不能继续进行预测')
    data['quaternion'] = r.as_quat()
    return data


def run_single():
    parser = argparse.ArgumentParser(description='D任务：官方TartanAir惯性数据短时预测')
    parser.add_argument('--data', type=Path, default=None, help='官方惯性压缩包路径，默认按难度自动选择')
    parser.add_argument('--sequence', default='P000', help='轨迹编号，默认P000')
    parser.add_argument('--difficulty', choices=['easy', 'hard'], default='easy', help='轨迹难度')
    parser.add_argument('--slam-states', type=Path, help='实际SLAM完整状态CSV；不填时为理想初始化')
    parser.add_argument('--slam-metadata', type=Path, help='实际SLAM状态的坐标与数据说明JSON')
    parser.add_argument('--match-slam-starts', action='store_true', help='只复用实际SLAM的起点，以真值初始化做配对上界')
    parser.add_argument('--summary-only', action='store_true', help='理想对照仅保存CSV/JSON，避免重复生成展示素材')
    parser.add_argument('--output', type=Path, help='指定新输出目录；实际SLAM结果不覆盖已有目录')
    args = parser.parse_args()
    archive = args.data or ROOT.parent / f'data_D/downloads/ArchVizTinyHouseDay_{args.difficulty}_imu.zip'
    data = load_sequence(archive, args.sequence, args.difficulty)
    t = data['imu_time']
    dt = float(np.median(np.diff(t)))
    if bool(args.slam_states) != bool(args.slam_metadata):
        raise ValueError('实际SLAM状态和元数据必须同时提供')
    has_slam = args.slam_states is not None
    if args.match_slam_starts and not has_slam:
        raise ValueError('配对理想上界需要提供实际SLAM状态与元数据')
    actual = has_slam and not args.match_slam_starts
    init_mode = 'actual_slam' if actual else 'ideal_gt'
    output = args.output or ROOT / 'results/D' / f'{"slam_" if actual else ""}{args.difficulty}_{args.sequence}'
    output.mkdir(parents=True, exist_ok=True)
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    # 三档参数均为人工敏感性设置；标准差按每个采样点定义，不是噪声密度。
    # 每段预测的固定偏置只在起点抽样一次；白噪声逐采样点独立生成。
    noise = {'无额外噪声': [0, 0, 0, 0],
             '低噪声': [0.002, 0.02, 0.0005, 0.01],
             '高噪声': [0.01, 0.10, 0.003, 0.05]}
    durations = [0.5, 1.0, 2.0, 3.0]
    starts = np.linspace(np.searchsorted(t, t[0] + 0.5),
                         np.searchsorted(t, t[-1] - 3.0) - 1, 10).astype(int)
    initial_states, histories = {}, {}
    gravity = [0, 0, 9.8]
    meta = {}
    if has_slam:
        sys.path.insert(0, str(Path(__file__).resolve().parent / 'integration'))
        from slam_state import read_states, last_trusted_state
        meta = json.loads(args.slam_metadata.read_text(encoding='utf-8-sig'))
        expected = dict(source='orb_slam3', environment='ArchVizTinyHouseDay',
                        difficulty=args.difficulty, sequence=args.sequence,
                        body_frame='tartanair_imu', pose_convention='body_to_world',
                        quaternion_order='xyzw', units='SI', time_offset_to_dataset_s=0)
        if any(meta.get(k) != v for k, v in expected.items()):
            raise ValueError('SLAM元数据与输入数据、时间或机体坐标约定不一致')
        slam_gravity = np.asarray(meta['gravity_world'], float)
        if slam_gravity.shape != (3,) or not np.isfinite(slam_gravity).all() or not 9 < np.linalg.norm(slam_gravity) < 10:
            raise ValueError('SLAM重力向量无效')
        if actual:
            gravity = slam_gravity
        states = read_states(args.slam_states)
        candidates = []
        for stamp in states.loc[(states.valid == 1) & (states.imu_initialized == 1), 'timestamp']:
            index = int(np.argmin(abs(t - stamp)))
            if abs(t[index] - stamp) > 1e-7 or stamp + 3 > t[-1] + 1e-8:
                continue
            state, history = last_trusted_state(states, stamp)
            history = history[history.timestamp >= stamp - 2]
            # 配准只用于离线评分。旋转由历史姿态求得，不靠两个位置点推断旋转。
            # 至少两帧，使用当前坐标版本内全部可用的失效前历史，禁止借用未来帧。
            if len(history) < 2 or history.timestamp.iloc[-1] - history.timestamp.iloc[0] < 0.09:
                continue
            candidates.append(index)
            initial_states[index], histories[index] = state, history
        if len(candidates) < 10:
            raise ValueError(f'只有{len(candidates)}个合法实际起点，不足10个；不能用真值补齐')
        starts = np.asarray(candidates)[np.linspace(0, len(candidates)-1, 10).round().astype(int)]
    if len(set(starts)) != 10 or starts[0] < 0:
        raise ValueError('轨迹长度不足以选择10个起点')
    rows = []
    timing_rows = []
    example = None
    init_label = '实际SLAM初始化' if actual else '真值理想初始化上界'
    for level, scales in noise.items():
        for seed in [0, 1, 2]:
            for start_id, start in enumerate(starts):
                count = int(round(3 / dt)) + 1
                stop = start + count
                segment = slice(start, stop)
                times = t[segment]
                rng = np.random.default_rng(np.random.SeedSequence([seed, start_id]))
                gs, acs, gbs, abs_ = scales
                gyro = data['gyro'][segment] + rng.normal(0, gbs, 3) + rng.normal(0, gs, (count, 3))
                accel = data['acc'][segment] + rng.normal(0, abs_, 3) + rng.normal(0, acs, (count, 3))
                bg, ba = np.zeros(3), np.zeros(3)
                if actual:
                    state = initial_states[start]
                    p0 = state[['px', 'py', 'pz']].to_numpy(float)
                    v0 = state[['vx', 'vy', 'vz']].to_numpy(float)
                    r0 = Rotation.from_quat(state[['qx', 'qy', 'qz', 'qw']].to_numpy(float))
                    bg = state[['bgx', 'bgy', 'bgz']].to_numpy(float)
                    ba = state[['bax', 'bay', 'baz']].to_numpy(float)
                else:
                    # 只有理想上界分支才使用起点真值；正式分支全部来自实际SLAM。
                    p0, v0 = data['pos_global'][start], data['vel_global'][start]
                    r0 = Rotation.from_quat(data['quaternion'][start])
                began = perf_counter()
                prediction = predict_imu(times, gyro, accel, p0,
                                         r0.as_quat(), v0,
                                         gyro_bias=bg, accel_bias=ba, gravity=gravity)
                prediction_ms = (perf_counter() - began) * 1000
                timing_rows.append(dict(noise=level, seed=seed, start_time=float(times[0]),
                                        sample_count=len(times), prediction_ms=prediction_ms,
                                        mean_step_ms=prediction_ms / (len(times) - 1)))
                # 预测结束后才读取对应真值计算误差；不能用整段对齐消除漂移。
                evaluated_position = prediction['positions']
                evaluated_rotation = Rotation.from_quat(prediction['quaternions'])
                if actual:
                    history = histories[start]
                    hi = np.searchsorted(t, history.timestamp.to_numpy() - 1e-8)
                    if not np.allclose(t[hi], history.timestamp.to_numpy(), atol=1e-7):
                        raise ValueError('历史帧不在IMU时间网格上')
                    align_rotation = (Rotation.from_quat(data['quaternion'][hi]) *
                                      Rotation.from_quat(history[['qx', 'qy', 'qz', 'qw']].to_numpy()).inv()).mean()
                    align_translation = np.mean(data['pos_global'][hi] - align_rotation.apply(
                        history[['px', 'py', 'pz']].to_numpy()), axis=0)
                    evaluated_position = align_rotation.apply(prediction['positions']) + align_translation
                    evaluated_rotation = align_rotation * evaluated_rotation
                ep = np.linalg.norm(evaluated_position - data['pos_global'][segment], axis=1)
                er = np.rad2deg((Rotation.from_quat(data['quaternion'][segment]).inv() *
                                evaluated_rotation).magnitude())
                # “维持两秒”要求整个片段满足阈值，而不仅是第二秒末尾恰好满足。
                within = (ep <= 1) & (er <= 10)
                strict_within = (ep <= 0.5) & (er <= 5)
                for duration in durations:
                    end = int(round(duration / dt))
                    rows.append(dict(sequence=args.sequence, difficulty=args.difficulty,
                                     start_time=float(times[0]),
                                     start_speed_mps=float(np.linalg.norm(v0)),
                                     start_angular_speed_radps=float(np.linalg.norm(data['gyro'][start])),
                                     duration_s=duration, noise=level, seed=seed,
                                     init_mode=init_mode,
                                     position_error_m=float(ep[end]),
                                     rotation_error_deg=float(er[end]),
                                     continuous_within_1m_10deg=bool(within[:end + 1].all()),
                                     continuous_within_05m_5deg=bool(strict_within[:end + 1].all()),
                                     within_1m_10deg=bool(ep[end] <= 1 and er[end] <= 10)))
                if level == '高噪声' and seed == 0 and start_id == 5:
                    example = (times, prediction, data['pos_global'][segment], ep, er, evaluated_position)

    frame = pd.DataFrame(rows)
    timing = pd.DataFrame(timing_rows)
    frame.to_csv(output / 'outage_error.csv', index=False, encoding='utf-8-sig')
    summary = frame.groupby(['noise', 'duration_s'], sort=False).agg(
        position_median_m=('position_error_m', 'median'),
        position_p90_m=('position_error_m', lambda x: x.quantile(0.9)),
        rotation_median_deg=('rotation_error_deg', 'median'),
        rotation_p90_deg=('rotation_error_deg', lambda x: x.quantile(0.9)),
        threshold_pass_fraction=('within_1m_10deg', 'mean'),
        continuous_pass_fraction=('continuous_within_1m_10deg', 'mean'),
        strict_continuous_pass_fraction=('continuous_within_05m_5deg', 'mean')).reset_index()
    settings = dict(dataset='TartanAir V2', environment='ArchVizTinyHouseDay', difficulty=args.difficulty,
                    sequence=args.sequence, init_mode=init_mode, seeds=[0, 1, 2],
                    start_count=10, start_times=t[starts].tolist(), imu_hz=1 / dt, gravity=list(gravity),
                    orientation='内禀XYZ欧拉角，弧度，机体到世界',
                    noise_columns=['角速度逐点标准差rad/s', '加速度逐点标准差m/s²',
                                   '固定陀螺偏置标准差rad/s', '固定加速度偏置标准差m/s²'],
                    noise=noise, sha256=HASHES[args.difficulty],
                    summary=summary.to_dict(orient='records'),
                    runtime=dict(segment_count=len(timing), measurements=timing_rows,
                                 mean_three_second_prediction_ms=float(timing.prediction_ms.mean()),
                                 p90_three_second_prediction_ms=float(timing.prediction_ms.quantile(0.9)),
                                 mean_step_ms=float(timing.mean_step_ms.mean()),
                                 scope='仅积分函数耗时，不含下载、读取、评估、绘图；不代表端到端实时性能'),
                    source='https://huggingface.co/datasets/theairlabcmu/tartanair2',
                    note=f'{init_label}；官方仿真IMU；噪声档位为人工设置；失效后无真值纠偏；窗口可能重叠',
                    versions=dict(numpy=np.__version__, pandas=pd.__version__, matplotlib=matplotlib.__version__))
    if actual:
        settings.update(slam_metadata=meta, state_file_sha256=hashlib.sha256(args.slam_states.read_bytes()).hexdigest(),
                        initial_states=[initial_states[s].to_dict() for s in starts],
                        alignment='仅失效前同地图、同坐标版本的可用历史做旋转和平移配准，不缩放',
                        alignment_history=[dict(start_time=float(t[s]), count=len(histories[s]),
                                                first=float(histories[s].timestamp.iloc[0]),
                                                last=float(histories[s].timestamp.iloc[-1])) for s in starts],
                        limitations=f'当前实际SLAM仅{args.difficulty}/{args.sequence}；10个起点集中在{t[starts[0]]:.2f}至{t[starts[-1]]:.2f}秒且窗口重叠；不代表10条独立轨迹')
    elif has_slam:
        settings.update(paired_with='actual_slam',
                        reference_state_file_sha256=hashlib.sha256(args.slam_states.read_bytes()).hexdigest(),
                        note='与实际SLAM实验使用完全相同的10个起点、噪声参数和种子；本分支仍为真值理想初始化上界')
    (output / 'metrics.json').write_text(json.dumps(settings, ensure_ascii=False, indent=2), encoding='utf-8')

    if args.summary_only:
        print(f'{args.difficulty}/{args.sequence}：{init_label}，360条误差记录，已保存CSV/JSON。')
        return

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for level in noise:
        part = summary[summary.noise == level]
        for axis, median, p90 in [(axes[0], 'position_median_m', 'position_p90_m'),
                                  (axes[1], 'rotation_median_deg', 'rotation_p90_deg')]:
            axis.plot(part.duration_s, part[median], marker='o', label=level)
            axis.fill_between(part.duration_s, part[median], part[p90], alpha=0.12)
    for axis, ylabel in zip(axes, ['位置误差（米）', '朝向误差（度）']):
        axis.set(xlabel='中断时间（秒）', ylabel=ylabel, xticks=durations)
        axis.grid(alpha=0.3)
        axis.legend()
    fig.suptitle(f'TartanAir {args.difficulty}/{args.sequence}：{init_label}，实线为中位数，阴影到90%分位数')
    fig.tight_layout()
    fig.savefig(output / '误差随时间变化.png', dpi=180)
    plt.close(fig)

    times, prediction, truth, ep, er, evaluated_position = example
    # 保存一个完整预测片段，供E成员接入；文件中不混入真值。
    np.savetxt(output / 'example_prediction.tum',
               np.column_stack([times, prediction['positions'], prediction['quaternions']]),
               fmt='%.10f', header='时间戳 位置x 位置y 位置z 四元数x 四元数y 四元数z 四元数w')
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(truth[:, 0], truth[:, 1], label='真实轨迹')
    axes[0].plot(evaluated_position[:, 0], evaluated_position[:, 1], '--', label='IMU预测（仅离线坐标配准）' if actual else 'IMU预测')
    axes[0].scatter(truth[0, 0], truth[0, 1], label='失效起点')
    axes[0].set(xlabel='世界X（米）', ylabel='世界Y（米）', title='水平轨迹投影')
    axes[0].axis('equal')
    axes[1].plot(times - times[0], ep, label='三维位置误差')
    axes[1].set(xlabel='中断时间（秒）', ylabel='位置误差（米）', title='误差增长过程')
    for axis in axes:
        axis.legend()
        axis.grid(alpha=0.3)
    fig.suptitle(f'{args.difficulty}/{args.sequence}高噪声示例，{init_label}，无真值纠偏')
    fig.tight_layout()
    fig.savefig(output / '典型预测轨迹.png', dpi=180)
    plt.close(fig)
    pd.DataFrame(dict(timestamp=times, state='IMU_PREDICTION', source='imu_prediction',
                      elapsed_s=times-times[0], position_error_m=ep, rotation_error_deg=er)).to_csv(
                          output / 'prediction_state.csv', index=False)
    print(f'数据读取及坐标验证通过：{args.sequence}，{len(t)}条IMU，{1/dt:.0f}赫兹')
    print('已完成：3档噪声×10个起点×3个种子，在4个时长处评估，共360条误差记录')
    print(f'2秒结果（{init_label}）：')
    print(summary[summary.duration_s == 2].to_string(index=False))
    print(f'结果目录：{output}')


def run_all():
    parser = argparse.ArgumentParser(description='运行D实验及统一汇总')
    parser.add_argument('--slam-states', type=Path, default=ROOT/'results/D/slam_easy_P000/slam_states.csv', help='实际SLAM完整状态CSV')
    parser.add_argument('--slam-metadata', type=Path, default=ROOT/'results/D/slam_easy_P000/slam_metadata.json', help='实际SLAM坐标与数据说明JSON')
    args = parser.parse_args()
    if bool(args.slam_states) != bool(args.slam_metadata):
        raise ValueError('SLAM状态与元数据必须同时提供')
    actual = args.slam_states is not None
    module = Path(__file__).resolve().parent
    result, presentation = ROOT / 'results/D', ROOT / 'presentation/D'
    result.mkdir(parents=True, exist_ok=True)
    presentation.mkdir(parents=True, exist_ok=True)
    runs, records = [], []
    # 测试和原始运行输出集中在同一个日志，不再产生多份日志。
    jobs = [
            (module / 'tests/check_integrator.py', []),
            (module / 'tests/check_slam_state.py', []),
            (module / 'run_experiment.py', ['--difficulty', 'easy']),
            (module / 'run_experiment.py', ['--difficulty', 'hard'])
        ]
    if actual:
        # 公平比较：easy的理想上界也取实际SLAM可用的同10个起点，逐窗口复用噪声与种子。
        jobs[2] = (module / 'run_experiment.py', ['--slam-states', str(args.slam_states),
                                                '--slam-metadata', str(args.slam_metadata), '--match-slam-starts', '--summary-only'])
        jobs[3][1].append('--summary-only')
        jobs.append((module / 'run_experiment.py', ['--slam-states', str(args.slam_states),
                                                   '--slam-metadata', str(args.slam_metadata)]))
    with (result / '运行日志.txt').open('w', encoding='utf-8') as log:
        for script, options in jobs:
            command = [sys.executable, '-B', str(script), *(['--single'] if script.name == 'run_experiment.py' else []), *options]
            completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       encoding='utf-8', errors='replace',
                                       env=dict(__import__('os').environ, PYTHONIOENCODING='utf-8'))
            log.write('运行：' + ' '.join(command) + '\n' + completed.stdout + '\n')
            print(completed.stdout, end='')
            if completed.returncode:
                raise RuntimeError('运行失败，请查看results/D/运行日志.txt')
    folders = ['easy_P000', 'hard_P000'] + (['slam_easy_P000'] if actual else [])
    for name in folders:
        folder = result / name
        record = pd.read_csv(folder / 'outage_error.csv')
        if len(record) != 360:
            raise ValueError('单轨迹误差记录应为360条')
        records.append(record)
        runs.append(json.loads((folder / 'metrics.json').read_text(encoding='utf-8')))
    combined = pd.concat(records, ignore_index=True)
    combined.to_csv(result / 'outage_error.csv', index=False, encoding='utf-8-sig')
    manifest = dict(status='已完成P000实际SLAM初始化与理想上界对照' if actual else '仅理想初始化实验', record_count=len(combined), runs=runs,
                    code_sha256={p.relative_to(module).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in module.rglob('*') if p.is_file() and
                                 p.suffix in ['.py', '.ps1', '.json', '.h', '.cc', '.patch', '.yaml', '.txt'] and
                                 '__pycache__' not in p.parts},
                    pending=[] if actual else ['实际SLAM状态及正式初始化实验'])
    (result / 'metrics.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    # presentation/D只保留组长要求的图片与一份实现文档。
    display = result / ('slam_easy_P000' if actual else 'easy_P000')
    shutil.copy2(display / '误差随时间变化.png', presentation / '误差随时间变化.png')
    shutil.copy2(display / '典型预测轨迹.png', presentation / '预测轨迹示例.png')
    summary = pd.concat([pd.DataFrame(run['summary']).assign(difficulty=run['difficulty'], init_mode=run['init_mode'])
                         for run in runs], ignore_index=True)
    summary.to_csv(result / 'summary.csv', index=False, encoding='utf-8-sig')
    two = summary[(summary.duration_s == 2) & (summary.noise == '高噪声')]
    worst = combined[combined.duration_s == 3].sort_values('position_error_m').iloc[-1]
    text = [
        'D：IMU短时预测',
        '', '方法：从实际SLAM的位姿、速度和IMU偏置出发，用中点积分预测；中断后只使用IMU。',
        '配置：easy/P000实际初始化；easy/P000配对理想上界；hard/P000理想上界。',
        '每组3档噪声×10个运动起点×3个种子，评价0.5/1/2/3秒。',
        '', '2秒高噪声结果：',
        two[['difficulty', 'init_mode', 'position_median_m', 'position_p90_m',
             'rotation_median_deg', 'rotation_p90_deg']].to_string(index=False),
        '', '范围：实际起点7.5～8.4秒、窗口重叠；P90为经验分位数，不是置信区间。',
        '预测完成后仅用中断前历史做离线坐标配准；中断后无真值纠偏。',
        '', '关键文件：',
        'experiments/D_inertial_outage/README.md：代码用途和唯一运行命令。',
        'results/D/outage_error.csv：逐实验误差；summary.csv：中位数/P90/通过率。',
        'results/D/metrics.json：配置、随机种子、源码哈希和统计结果。',
        'results/D/slam_easy_P000/：真实初始状态、原始基线日志、预测轨迹和正式结果。',
        '本目录PPT：两页汇报，讲解放在幻灯片备注中。',
        '本目录两张PNG：误差曲线与预测漂移轨迹。'
    ]
    (presentation / '实现说明.txt').write_text('\n'.join(text), encoding='utf-8-sig')
    print(f'D实验汇总完成：{len(combined)}条误差记录，展示图片与实现说明已更新。')



if __name__ == '__main__':
    if '--single' in sys.argv:
        sys.argv.remove('--single')
        run_single()
    else:
        run_all()
