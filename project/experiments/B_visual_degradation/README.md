# B_visual_degradation

B 分支：对录制序列只替换 `rgb/` 做离线图像退化，跑 ORB-SLAM3 三种模式，看覆盖、失跟、ATE 随退化强度的变化。

- `visual_corrupt.py`：生成扰动序列。故障与档位在 `configs/B_visual.yaml`（version `B_visual_v1`），
  5 故障 × L1–L4，随机量只由 (seed, 故障名, 帧号或时段号) 决定。输出
  `<out-root>/<seq>__<fault>_<level>_s<seed>/`：`rgb/` 是退化后的帧，其余（depth、imu、GT、calib）
  全部软链接回原序列；附 `fault_table.csv`（逐帧参数、受影响面积、亮度、Laplacian 方差、ORB 点数）
  和 `fault_meta.json`（参数、配置 sha256）。`mosaic` 子命令出原图 vs 各档拼图，`--orb` 叠 ORB 点。
- `run_B.sh`：批量运行包装。设 `ulimit -c 0`，并把 TMPDIR/MPLCONFIGDIR/XDG_* 等缓存重定向到一个临时目录
  （默认 `ROOT/.scratch/`，可用环境变量 `SCRATCH_ROOT` 指定），退出时只删这个目录。
- `plot_B.py`：`table` → `results/B/visual_stress.csv`；`curves` → 各指标“强度—指标”曲线（误差棒 =
  种子间标准差，空心圈 = 有种子失败、× = 该档全部运行无可用值）；`case` → 失败案例（分段对齐的
  在线轨迹俯视图 + 状态时间轴 + 跟踪点数）。默认 `--lang en`，`--lang zh` 用 Noto Serif CJK。

矩阵（`configs/matrix/`）：

| 矩阵 | 序列 | 规模 | 用途 |
|---|---|---|---|
| `B_smoke.yaml` | office_easy_p004 | gaussian_blur/occlusion × L2,L4 × seed 0 × 3 模式 = 15 | 链路验证，不进正式结果 |
| `B_main.yaml` | office_easy_p004 | (5 故障 × L1–L4 + clean) × 3 种子 × 3 模式 = 189 | 验收主结果 |
| `B_hard.yaml` | office_hard_p004 | 同上 189 | 副结果（hard 基线本身大量重置） |

评测口径（`scripts/evaluate.py`）：主指标为 online 轨迹，按 (map_id, frame_epoch) 分段对齐
（MONOCULAR 用 Sim(3)，IMU 模式用 SE(3)），每段丢第一帧；有效帧 = state OK 且跟踪点数 ≥ 1；
恢复判据 `--recover-err 0.5`（偏宽松，同时报告 `n_unrecovered`）。矩阵里统一 `--rate 1`
（实时回放）和 `--recover-err 0.5`。

运行（并发数按机器 CPU 配额设定，实验时 B、C 合计不超过 4）：

```bash
cd ROOT
# 1) 生成 + 跑 + 评测；--cleanup 用完即删扰动序列
experiments/B_visual_degradation/run_B.sh configs/matrix/B_main.yaml 2 --cleanup
# 2) 中途中断可直接重跑同一条命令（断点续跑）；口径更新后用 --reevaluate 只重评
experiments/B_visual_degradation/run_B.sh configs/matrix/B_main.yaml 1 --reevaluate
# 3) 汇总 + 出图
env_data/bin/python scripts/aggregate.py results/B/runs --out results/B/visual_stress_runs.csv
env_data/bin/python experiments/B_visual_degradation/plot_B.py all \
    --runs-csv results/B/visual_stress_runs.csv --runs-dir results/B/runs
```

产物：`results/B/visual_stress_runs.csv`（每次运行一行）、`results/B/visual_stress.csv`
（按 序列×故障×档位×模式 汇总）、`results/B/figures/`（曲线、拼图、失败案例）、
`presentation/B/`（精选图和文档）。自测：`experiments/tests/selftest_corrupt.py`（与 C 共用，43 项）。
