# B 视觉退化：建议 PPT 页（对应 PROJECT_PLAN 2.6）

> 状态（2026-11-26 13:20）：easy 主序列 189 次已全部完成（183 ok + 6 init_failed）；hard 副序列不在验收范围内，停在 96/189（90 ok + 6 init_failed），只作现象描述。数字来源：`results/B/visual_stress.csv`、`results/B/visual_stress_runs.csv`。

## 第 2 页（背景，B 提供相机失效素材）
- **标题**：相机在哪些条件下会失效
- **核心结论**：模糊、低照度、遮挡和弱纹理会让 ORB 特征快速减少。
- **用图**：`figures/01_mosaic_faults_orb_easy.png`（裁出 1–2 行即可）
- **讲稿要点**：
  1. 5 种故障各 4 档，全部是对录制图像的离线退化；
  2. 绿点是 ORB 关键点，同一帧上高斯模糊从 L1 到 L4，ORB 点数从 987 降到 15（hard 序列第 300 帧）；
  3. 遮挡和纹理抹除在 L2 时 ORB 点数几乎不变，因为 ORB 会在剩余区域补足点数，影响要看 SLAM 实际跟踪结果；
  4. 纹理抹除只是图像级近似，不代表真实的白墙几何。

## 第 5 页（实验设置，B 提供扰动协议）
- **标题**：统一的视觉退化协议
- **核心结论**：所有方法看到的是同一组确定性故障（固定种子 0/1/2）。
- **用图**：`实现说明.md` 3.1 节的参数表（可转成 PPT 表格），以及 `figures/02_mosaic_faults_orb_hard.png`
- **讲稿要点**：
  1. 5 故障 × L0–L4 × 3 种子 × 3 模式，每条序列 189 次运行；
  2. 只替换 RGB，深度、IMU、GT 都和原序列相同；
  3. 有效帧 = state OK 且至少 1 个跟踪点；主指标是 online 轨迹按 (map_id, frame_epoch) 分段对齐的 ATE，同时报告覆盖率、段数和失跟事件数；
  4. 恢复判据：连续 3 帧有效且误差 ≤ 0.5 m（偏宽松，同时报告未恢复事件数）；覆盖率 <0.1 的运行不计入 ATE。

## 第 7 页（视觉与深度退化，B 部分）
- **标题**：视觉退化下谁先失效
- **核心结论**：RGB-D+IMU 几乎不失效（5 种故障 L1–L4 覆盖率基本 ≥0.83，只有高斯模糊 L4 崩到 0.03）；纯单目和单目+IMU 在 L3 前后越过 0.5，且单目+IMU 在强遮挡/纹理抹除时不比纯单目稳。
- **用图**：`figures/03_coverage_vs_level_easy.png`、`04_lost_events_vs_level_easy.png`、`05_ate_vs_level_easy.png`、`06_case_gaussian_blur_L4_IMU_MONOCULAR.png`、`07_case_gaussian_blur_L4_IMU_RGBD.png`；hard 只作现象描述，用 `08_coverage_vs_level_hard.png`、`09_lost_events_vs_level_hard.png`、`10_segments_vs_level_hard.png`
- **讲稿要点**：
  1. 失效拐点（覆盖率首次 <0.5）：高斯模糊最早——单目+IMU 在 L2（0.442）、纯单目在 L3（0.149）；运动模糊、遮挡、纹理抹除的两种单目模式都在 L3 跌破 0.5；低照度/对比度到 L4 仍在 0.54–0.55。RGB-D+IMU 只在高斯模糊 L4 失效（0.032）。
  2. 初始化失败 6 次，全部是高斯模糊 L4 的纯单目与单目+IMU（各 3 个种子）；同图下 RGB-D+IMU 仍能初始化，但只剩 10 段、3.2% 有效帧。
  3. IMU 不是单调增益：高斯模糊 L2/L3、遮挡 L3/L4、纹理抹除 L3/L4 上，单目+IMU 覆盖率反而低于纯单目（如纹理抹除 L4：0.142 对 0.250），失跟事件更多（遮挡 L4：45.3 对 21.0）；种子间方差也大（遮挡 L1/L2 标准差 0.22/0.32）。
  4. ATE 必须和段数、aligned coverage 一起读：纹理抹除 L4 纯单目 ATE 0.076 m 看着很小，但只有 0.120 的 aligned coverage、平均 113 段；遮挡 L4 纯单目 119 段、21 次失跟。强扰动下以覆盖率和失跟为主。
  5. 回环会放大 final ATE：遮挡 L4 RGB-D+IMU 的 final ATE 2.736 m，而 online ATE 0.060 m；主结论用 online ATE。
  6. hard 副序列不在验收范围内，停在 96/189（90 ok + 6 init_failed）：基线覆盖率只有 0.450 / 0.302 / 0.624，L0 已有 86.7 / 98.7 / 154.3 段，退化后的低覆盖率与基线失败分不开；gaussian/motion 上覆盖率随强度继续下降，6 次 init_failed 都是 gaussian L4 的两种单目模式，只作现象描述。
