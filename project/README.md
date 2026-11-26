# 给 A 的参考材料（来自 B/C 组）

这个分支**不是 A 的交付**，是 B/C 组在搭建自己实验时顺带整理出来、A 可能用得上的东西。A 自己的分支是 `experiment/a-baselines`，这里没有动它。

## 可以直接借鉴的

| 内容 | 位置 |
|---|---|
| 在 conda 环境里编译 ORB-SLAM3（含 Pangolin）的完整步骤和踩过的坑 | `SETUP.md` 第 1 节；`presentation/A/实现说明.md` 第 2 节 |
| 统一数据格式与 TartanAir V2 下载、转换脚本 | `SETUP.md` 第 3 节；`scripts/download_tartanair.py`、`scripts/convert_tartanair.py` |
| 支持 MONOCULAR / IMU_MONOCULAR / RGBD / IMU_RGBD 四种模式、不开可视化、逐帧输出跟踪状态的运行入口 | `scripts/run_orbslam.sh`、`scripts/run_orbslam.cc` |
| 评测脚本（ATE/RPE、覆盖率、失跟、恢复时间）及其自测 | `scripts/evaluate.py`、`scripts/tests/test_evaluate.py` |
| 在线轨迹要按 (map_id, frame_epoch) 分段对齐：IMU 初始化、回环时 ORB-SLAM3 会改变世界坐标系，整条直接对齐 ATE 会大一个数量级 | `presentation/A/实现说明.md` 第 6 节 |
| 并发对结果的影响：CPU 不够时 ORB-SLAM3 跟不上 10 Hz，覆盖率会明显下降 | `presentation/A/实现说明.md` 第 7 节、`presentation/A/figures/03_concurrency.png`、`results/A/T1b_concurrency/` |
| hard 序列上 IMU 模式反复重置的根因（IMU 未初始化完一失跟就清空地图，而初始化至少要 10 个关键帧） | `presentation/A/实现说明.md` 第 8 节 |

## 没有完成的

B/C 组只做到自己实验需要的程度，之后就停了：

- `presentation/A/实现说明.md` 第 9 节和 `slides.md` 第 6 页的基线数字是“未完成”。B/C 的实验里已经有无扰动运行（`experiment/b-visual`、`experiment/c-depth` 分支 `results/<B|C>/runs_archive/` 里名字带 `clean` 的运行），可以用 `experiments/A_baselines/collect_baseline.py` 汇总，但 A 的正式基线建议按 A 自己的实验矩阵重跑。
- `experiments/A_baselines/T1b/calib/` 下的调参配置没有用过。
