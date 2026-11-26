# C_depth_degradation

C 分支：对录制序列只替换 `depth/` 做离线深度退化，跑 ORB-SLAM3 的 RGBD、IMU_RGBD，另加无扰动
IMU_MONOCULAR 作 RGB+IMU 对照。看无效比例、测距噪声、整帧中断、整条无深度时系统是正常运行、
初始化失败还是反复重置。

- `depth_corrupt.py`：生成扰动序列。故障与档位在 `configs/C_depth.yaml`（version `C_depth_v2`）：
  invalid_pixels P00/P20/P50/P80（在原本有效像素中置 0 的比例）、depth_noise N1–N4（相对噪声
  σ = r·z，r = 1/2/5/10%）、outage O05/O10/O20（t=10 s 起整帧深度置 0 0.5/1/2 s）、depth_off（整条置 0）。
  随机量只由 (seed, 故障名, 帧号) 决定；输出 `<out-root>/<seq>__<fault>_<level>_s<seed>/`，
  只有 `depth/` 是新文件，其余软链接回原序列。`mosaic` 子命令出原始/腐化深度拼图。
- `plot_C.py`（`--lang en` 默认）：`--runs results/C/depth_ablation_runs.csv --out results/C/figures`。
  每序列出 `01_invalid_ratio_*`（无效比例 — ATE / 覆盖率 / 分段数，三子图）、`02_noise_*`
  （噪声 r — 同上）、`03_outage_*`（中断时长 — 窗口内有效帧比例 / 失跟）、`04_status_*`（各档位运行结局）。
  分段数子图：实线全部 (map_id, frame_epoch) 段，虚线参与 ATE 的段；无效比例图 x=0 合并 clean 与 P00。
  `03_outage` 的上子图由 `--runs-dir`（默认 `results/C/runs`）读每次运行的 `tracking.csv` 逐帧计算
  `[outage_start_s, outage_end_s)` 内有效帧比例（规则同 `scripts/evaluate.py`），1.0 = 中断期间跟踪未断。
- `make_tables.py`：把 aggregate 的汇总 CSV 转成 Markdown 表（写文档时用）。

矩阵（`configs/matrix/C_depth.yaml`）：easy + hard × (clean + 12 故障档位) × 3 种子 ×
{RGBD, IMU_RGBD}，加无扰动 IMU_MONOCULAR 对照 = 162 次运行。正式运行 `--workers 2
--corrupt-workers 1 --cleanup`（B、C 合计并发不超过 4）。

评测口径（`scripts/evaluate.py`）：主指标 online 轨迹，按 (map_id, frame_epoch) 分段对齐
（RGBD/IMU 均 SE(3)），每段丢第一帧；有效帧 = state OK 且跟踪点数 ≥ 1；恢复判据
`--recover-err 0.5`（偏宽松，同时报告 `n_unrecovered`）；final 轨迹 ATE 作副指标，与回环一起报告。

运行：

```bash
cd ROOT
env_data/bin/python scripts/run_matrix.py configs/matrix/C_depth.yaml --workers 2 --corrupt-workers 1 --cleanup
env_data/bin/python scripts/run_matrix.py configs/matrix/C_depth.yaml --reevaluate      # 口径更新后重评
env_data/bin/python scripts/aggregate.py results/C/runs --out results/C/depth_ablation_runs.csv \
    --group-out results/C/depth_ablation.csv
env_data/bin/python experiments/C_depth_degradation/plot_C.py --runs results/C/depth_ablation_runs.csv \
    --out results/C/figures --runs-dir results/C/runs
env_data/bin/python experiments/C_depth_degradation/depth_corrupt.py mosaic --seq data/seqs/office_easy_p004 \
    --frame 600 --out results/C/figures/05_depth_mosaic_easy.png
```

结果（2026-11-26 重评后，`results/C/depth_ablation.csv`）：162 次 = 148 ok + 12 init_failed
（全部 depth_off）+ 2 run_failed（easy RGBD 的 P20_s2、P80_s2，exit 134，Sophus NaN）。

- easy 无扰动：IMU_RGBD ATE 0.036 m / 覆盖率 0.998；RGBD 0.097 m / 0.997；IMU_MONOCULAR 0.040 m / 0.916。
- 无效像素 0–50%：两种模式 ATE 基本不变；P80 时 RGBD 表面 ATE 0.026 m 是 98.7 段（无扰动 16.7）
  的伪影，final 0.083 m。IMU_RGBD 在 P80 失跟从 0 涨到 7 次。
- 噪声：r ≤ 5% 对 IMU_RGBD 无可分辨影响；r = 10% 时 ATE 0.058 m（比无扰动高 62%）。
- 中断：easy 的 18 次中断运行在中断窗口内有效帧比例全为 1.000（跟踪没断，窗口后恢复时间 0.0 s）；
  RGBD 整次运行的失跟组均 1–2 次（单次 0–4 次），逐次核对都发生在窗口之外；hard 的窗口内比例
  RGBD 0.83–1.00、IMU_RGBD 0.12–0.60，与基线反复重置混在一起。
- depth_off：12/12 初始化失败，覆盖率 0；逐帧是"每帧建空地图、下一帧就丢"（easy RGBD s0：
  609 帧 OK 全部 0 个跟踪点，交替 609 帧 LOST），不会退回单目惯性。
- hard 的 IMU 模式没有 final 指标（保存时 `Map 0 has 0 KFs`，wait_status=139）。
- 回环：easy IMU 模式 11 次回环中 9 次的 final ATE 被拉到 6.24–8.04 m（另 2 次 0.040、0.053 m，
  在线 0.024–0.046 m）；RGBD 回环 final 0.07–0.48 m。

产物：`results/C/depth_ablation_runs.csv`（每次运行一行）、`results/C/depth_ablation.csv`
（按 序列×故障×档位×模式 汇总）、`results/C/figures/`、`presentation/C/`（精选图、实现说明、slides）。
详细发现、逐项来源和局限见 `presentation/C/实现说明.md`。
