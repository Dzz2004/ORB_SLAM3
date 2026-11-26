# B：视觉退化：PPT 素材与说明

课程项目 [Dzz2004/ORB_SLAM3](https://github.com/Dzz2004/ORB_SLAM3) 的 **B：视觉退化** 部分，需求见 `docs/PROJECT_PLAN.md` 第四节。数据是 TartanAir V2 Office/P004 的 easy 和 hard 两条序列，算法是官方 ORB-SLAM3，没有改源码。

高斯模糊、运动模糊、低照度/对比度、遮挡、纹理抹除，每种 4 档（L1–L4），模式为 MONOCULAR、IMU_MONOCULAR、IMU_RGBD。

| 文件 | 内容 |
|---|---|
| `B_汇报.pptx`（同内容 `B_汇报.pdf`） | 11 页：结论、扰动设置、实验口径、谁先失效、IMU 不一定更稳、ATE 怎么读、失败案例、hard 序列、局限、总结；讲稿在每页备注里 |
| `figures/` | PPT 用图（英文标注），序号即建议出场顺序 |
| `实现说明.md` | 研究问题、环境与数据、方法、运行命令、结果表、主要发现、局限 |
| `slides.md` | 对应 PROJECT_PLAN 2.6 节第 2、5、7 页的素材建议 |
| `_build/` | PPT 生成脚本（pptxgenjs） |

## 完成情况（2026-11-26）

| 部分 | 规模 | 结果 |
|---|---|---|
| B 主实验（easy） | 5 种故障 × L0–L4 × 3 种子 × 3 种模式 = 189 次 | 183 次正常，6 次初始化失败（都在高斯模糊 L4，纯单目和单目+IMU） |
| B hard 序列 | 同上，189 次 | **部分完成 96/189**，只描述现象、不下趋势结论 |

每次运行的原始记录打包在 `results/B/runs_archive/`：`runs_metrics.tar.gz`（每次运行的 metrics.json、运行信息、坐标系切换事件）和 `runs_logs.tar.gz`（逐帧 tracking.csv、在线/优化后轨迹、run.log），解压到 `results/B/runs/` 即可用 `scripts/aggregate.py` 重新汇总。文件里原机器的绝对路径已替换为 `ROOT`。

所有数字来自 `results/B/visual_stress.csv`、`results/B/visual_stress_runs.csv` 和逐次运行的 `metrics.json`，文档中均注明出处，并已用 `experiments/T7_audit/` 的脚本逐项核对。

## 读结果前需要知道的口径

1. **有效帧** = 跟踪状态 OK 且至少跟踪到 1 个地图点。整条无深度时，ORB-SLAM3 每帧都会建一张空地图并报告 OK，所以要加上第二个条件。
2. **在线 ATE 按段对齐**：ORB-SLAM3 在重置地图、IMU 初始化、回环、地图合并时都会改变世界坐标系，所以在线轨迹按 (map_id, frame_epoch) 分段，每段单独对齐，并丢弃每段的第一帧。**段数越多，ATE 越偏小**，因此 ATE 必须和段数、覆盖率一起看。例如纹理抹除 L4 的纯单目 ATE 只有 0.076 m，但平均 113 段、只有 12% 的帧参与对齐，这个 ATE 不能说明更准。
3. **覆盖率低于 0.1 的运行不计入 ATE**，初始化失败的运行覆盖率按 0 计。
4. **final ATE 是副指标**：ORB-SLAM3 保存的优化后轨迹只保留最大的一张地图；IMU 模式回环后，这条轨迹会被惯性回环矫正拉长，final ATE 会变成 6–8 m，而在线 ATE 正常。
5. 每个设置跑 3 个种子，图上的误差棒是种子间标准差。ORB-SLAM3 是多线程的，同一配置重复运行也会有差异。
6. 容器只有 4 核 CPU 配额，全部实验同时最多跑 4 个 ORB-SLAM3，以保证每帧处理不超过 10 Hz 的时间预算。

## PPT 字体

西文 Times New Roman，中文华文行楷，写在主题字体里，可在 PowerPoint“设计 → 变体 → 字体”里一次性更换。生成时本机没有这两种字体，预览用的是替代字体，在 Windows 上打开时字宽会略有不同，建议打开后翻一遍。

## 复现

环境（ORB-SLAM3 编译、Python venv）和 TartanAir 数据的准备步骤见本分支根目录的 `SETUP.md`。代码和配置在 `scripts/`、`configs/`、`experiments/B_visual_degradation/`，步骤见 `实现说明.md` 的“运行命令”一节。原始数据和扰动序列不放进 git，用 `scripts/download_tartanair.py` 和 `scripts/convert_tartanair.py` 重新生成。重新生成 PPT：`node presentation/B/_build/build_B.js presentation/B 输出.pptx <pptx skill 目录>`（需要 node 和 pptxgenjs）。
