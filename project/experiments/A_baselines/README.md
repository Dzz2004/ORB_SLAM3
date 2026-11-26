# A_baselines

- `collect_baseline.py`：从 B/C 矩阵的 clean 运行（`results/B/runs`、`results/C/runs` 中的 `*__clean_s*__*`）只读汇总，生成 `results/A/baseline_runs.csv` 和 `baseline_metrics.csv`。
- `plot_baseline.py`：生成 `results/A/figures/`，包括 01 各模式指标柱状图、02 轨迹俯视图、03 并发影响图。`--lang en`（默认，按科研绘图惯例）输出英文图；`--lang zh` 使用 `env_docs/share/fonts/noto-serif-cjk-sc/NotoSerifCJKsc-Regular.otf` 输出中文图。模式名和 ATE/RPE 等缩写保持英文。
- `T1b/`：并发测试、frame_epoch 验收脚本（`check_jumps.py`、`split_test.py`），以及 hard 序列调参用的 calib 副本（未使用）。

运行（先重定向缓存）：

```bash
source scripts/docs_env.sh
env/bin/python -I experiments/A_baselines/collect_baseline.py
env/bin/python -I experiments/A_baselines/plot_baseline.py
cp results/A/figures/0*.png presentation/A/figures/
```
