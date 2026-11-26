## 研究问题：

在 TartanAir V2 上，ORB-SLAM3 的 RGB、RGB+IMU、RGB-D+IMU 三种模式在轨迹精度、
有效跟踪覆盖率、失跟次数、地图连续性和计算耗时方面有何差异？hard 场景发生地图重置时，
如何在不拼接不同坐标系的条件下进行可复现评估？

## 输入数据与实验配置：

- 数据集：TartanAir V2，`OldTownSummer`
- 主实验：easy `P000`，第 200–299 帧，20.0–29.9 s
- Holdout：easy `P002`，第 200–299 帧，20.0–29.9 s
- Hard 鲁棒性实验：hard `P001`，第 100–199 帧，10.0–19.9 s
- 扩展矩阵：当前下载包内 Easy/Hard 的 P000、P001、P002 全部轨迹
- 模式：`rgb`、`rgb_imu`、`rgbd_imu`
- 随机种子：`0`、`1`、`2`
- 相机：640×640 pinhole，`fx=fy=320`，`cx=cy=320`，无畸变
- IMU：100 Hz，固定 optical-to-NED `Tbc`
- ORB：1250 features，scale 1.2，8 levels
- OpenMP/OpenBLAS/MKL：固定单线程
- 对齐：RGB 使用 Sim(3)，融合模式使用 SE(3)
- RPE：1 m interval；hard 公共窗口补充实验使用 5-frame interval
- 完整窗口 ATE/RPE 仅在有效位姿属于单一 map 时计算

详细配置见：

```text
experiments/A_baselines/matrix.yaml
configs/A/tartanair.yaml
```

## 运行命令：

```bash
export A_ROOT=/path/to/ORB_SLAM3_A
export REPO_ROOT=/path/to/ORB_SLAM3

source "${REPO_ROOT}/scripts/A/activate_a_orb.sh"
bash "${REPO_ROOT}/scripts/A/build_pangolin.sh"
bash "${REPO_ROOT}/experiments/A_baselines/build_a.sh"
bash "${REPO_ROOT}/experiments/A_baselines/reproduce_baseline_v2.sh"
```

该命令运行 easy/P000、easy/P002、hard/P001 的三模式 × 三 seed 矩阵，并重新生成
hard 公共窗口、逐 map 分段、CSV/JSON 聚合结果和全部图。

## 输出路径（结果、图、日志）：

结果：

```text
results/A/baseline_metrics.csv
results/A/baseline_aggregate.csv
results/A/validation_metrics.csv
results/A/validation_aggregate.csv
results/A/hard_metrics.csv
results/A/hard_aggregate.csv
results/A/hard_common_metrics.csv
results/A/hard_common_aggregate.csv
results/A/hard_map_segments.csv
results/A/hard_map_segment_summary.csv
results/A/all_sequences_metrics.csv
results/A/all_sequences_aggregate.csv
results/A/available_sequence_trajectory_manifest.csv
```

每个 run 的原始结果、轨迹、tracking 日志和 `metrics.json`：

```text
results/A/baseline_final/easy/P000_window_200_299/
results/A/validation/P002/easy/
results/A/baseline_final/hard/P001_window_start100/
```

图和汇报材料：

```text
results/A/figures/
presentation/A/figures/
```

运行日志：

```text
results/A/baseline_final/easy/P000_window_200_299/*/run.log
results/A/validation/P002/easy/*/run.log
results/A/baseline_final/hard/P001_window_start100/*/run.log
```

完整报告：

```text
experiments/A_baselines/baseline_report.md
experiments/A_baselines/failure_cases.md
presentation/A/docs/
```

## 主要发现（含失败结果）：

- 全部 54 个 run 都保留 tracking 日志并有逐帧位姿；其中 32 个得到可作连续评测的
  主轨迹，18 个没有可用优化关键帧轨迹，4 个主轨迹跨 map、连续 ATE/RPE 记为 N/A。
- easy/P000：RGB+IMU 的 3 个 seed 均可计算 ATE，ATE 为 0.090 ± 0.057 m；
  但 1 m RPE 只有 2/3 seed 可计算，覆盖率 73.3%。
- easy/P001：RGB+IMU 的 3 个 seed 均可计算 ATE，ATE 为 0.300 ± 0.254 m；
  关键帧轨迹过稀，1 m RPE 在 3 个 seed 中均不可用，因此 ATE 和 RPE 分开记录。
- easy/P002：RGB+IMU 和 RGB-D+IMU 均优于 RGB，说明 easy 场景的融合收益存在场景依赖。
- hard/P001 全窗口覆盖率：RGB 50.3%、RGB+IMU 28.0%、RGB-D+IMU 60.0%。
- hard/P001 全窗口跨越不连续地图坐标系，因此 ATE/RPE 标记为 `N/A`，不拼接不同 map。
- hard/P001 预先固定的 12.9–14.9 s 单地图公共窗口中，RGB 有 1 个 seed 覆盖率为 0；
  该失败结果被保留。RGB 条件 ATE 为 0.281 m（2/3 seed），RGB+IMU 为 1.415 m，
  RGB-D+IMU 为 1.262 m。
- 逐 map 分段评估得到 RGB 4 个、RGB+IMU 0 个、RGB-D+IMU 3 个可评估连续片段。
- 扩展矩阵覆盖全部 6 条本地轨迹，共 54 个 run；新增 P001 Easy、P000 Hard 和
  P002 Hard 的失败结果均保留。
- `primary_trajectory_unavailable` 表示没有足够的关键帧导出结果，不等同于没有
  tracking；失败 run 仍输出逐帧 by-map 诊断图和完整覆盖率日志。
- 结果显示融合不保证在每条序列上优于 RGB；hard 场景主要暴露 IMU 初始化、跟踪丢失
  和地图连续性失败。

## 与其他分支的接口变化：

- 提供统一的 `manifest.csv`、`imu.csv`、`tracking.csv` 和 TUM 轨迹接口。
- `tracking.csv` 增加 `map_id`、`valid`、`state`、`quality_rgb`、`quality_depth`、
  `source` 和 `latency_ms`，供 B/C/D/E 复用。
- 统一主轨迹为 TartanAir body/NED 坐标下的优化关键帧轨迹。
- 增加 map-aware 评估：固定窗口跨地图时拒绝计算连续 ATE/RPE。
- hard 全窗口提供鲁棒性指标，公共单地图窗口提供条件精度，逐 map 分段单独对齐。
- 增加全部可用轨迹的统一汇总与逐模式轨迹图清单。
- B/C 可直接替换 `rgb_path` 或 `depth_path` 后复用 `prepare_sequence.py` 和
  `run_baseline.py`；时间戳、帧 ID 和顺序必须保持不变。
