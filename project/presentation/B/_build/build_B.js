// B visual-degradation deck. Usage: node build_B.js <presentation/B dir> <out.pptx> <pptx skill dir>
// Every number comes from presentation/B/slides.md and 实现说明.md (checked against results/B/visual_stress.csv).
const path = require("path");
const { makeDeck } = require("./deck_lib");
const [, , DIR, OUT, SKILL] = process.argv;
const fig = name => path.join(DIR, "figures", name);

const d = makeDeck("B：视觉退化下的 ORB-SLAM3", "ORB-SLAM3 · B 视觉退化");
const { pres, C } = d;

pres.addSection({ title: "开场" });
d.cover("开场", "B：图像变差时，谁先跟丢",
  "模糊 · 低照度 · 遮挡 · 纹理抹除　|　TartanAir V2 Office/P004　|　ORB-SLAM3 三种模式",
  "B 部分研究：图像质量变差时，纯单目、单目+IMU、RGB-D+IMU 哪个先失效，能不能测出稳定的失效转折点。");

let s = d.content("开场", "结论", "RGB-D+IMU 几乎不受视觉退化影响；两种单目模式大多在第 3 档失效");
d.card(s, 0.6, 1.9, 3.9, 2.0, "≥ 0.83", "RGB-D+IMU 在 5 种退化最强档下的覆盖率（唯一例外：高斯模糊 L4 为 0.032）", C.accent2);
d.card(s, 4.72, 1.9, 3.9, 2.0, "L3", "纯单目、单目+IMU 在运动模糊 / 遮挡 / 纹理抹除下覆盖率跌破 0.5 的档位", C.accent1);
d.card(s, 8.84, 1.9, 3.9, 2.0, "0.142 vs 0.250", "纹理抹除 L4 覆盖率：单目+IMU 反而低于纯单目", C.accent1);
d.bullets(s, [
  "高斯模糊最致命：单目+IMU 第 2 档就跌破 0.5，最强档时两种单目模式全部初始化失败",
  "低照度 / 对比度影响最小：到最强档仍有 0.54–0.55 的覆盖率",
  "强扰动下 ATE 会被切碎的短片段压低，应以覆盖率和失跟次数为主要结论",
], 0.6, 4.25, 12.1, 2.4, 16);
d.caption(s, "easy 序列，3 个种子均值。来源：results/B/visual_stress.csv", 0.6, 6.7, 12.1);
s.addNotes("先给结论。三张卡片分别是：RGB-D+IMU 最稳；单目系统的失效档位；加 IMU 反而更差的反例。");

pres.addSection({ title: "实验设置" });
s = d.content("实验设置", "5 种视觉退化，每种 4 档", "全部是对录好的图像做离线退化，只替换 RGB");
d.image(s, fig("01_mosaic_faults_orb_easy.png"), 1280 / 1314, 0.6, 1.75, 5.4, 4.85, "B mosaic easy");
d.table(s, [
  ["故障", "L1 / L2 / L3 / L4"],
  ["高斯模糊", "σ = 1 / 2 / 4 / 8 px"],
  ["运动模糊", "线核长 5 / 11 / 21 / 41 px"],
  ["低照度/对比度", "对比度 0.8 / 0.6 / 0.4 / 0.25，亮度 −20 / −45 / −70 / −95"],
  ["遮挡", "3 个黑色矩形，总面积 10 / 25 / 40 / 60%"],
  ["纹理抹除", "2 个区域，面积 15 / 30 / 50 / 75%，替换为强模糊"],
], 6.4, 1.85, 6.3, [1.7, 4.6], 12);
d.bullets(s, [
  "绿点是 ORB 关键点：hard 第 300 帧，高斯模糊 L1→L4 点数 987 → 15（拼图生成时的统计）",
  "纹理抹除是图像级近似，不等于真实白墙几何",
], 6.4, 4.95, 6.3, 1.7, 13);
d.caption(s, "图：figures/01_mosaic_faults_orb_easy.png", 0.6, 6.7, 5.4);
s.addNotes("拼图每行一种故障，从左到右强度递增。遮挡和纹理抹除在 L2 时 ORB 点数几乎不变，因为 ORB 会在剩下的区域补足点数。");

s = d.content("实验设置", "实验矩阵与评测口径", "所有模式看到的是同一组确定性故障（种子 0 / 1 / 2）");
d.table(s, [
  ["项目", "设置"],
  ["序列", "TartanAir V2 Office/P004 easy（1220 帧 / 122 s，10 Hz）为主；hard 为副"],
  ["模式", "MONOCULAR、IMU_MONOCULAR、IMU_RGBD"],
  ["规模", "5 种故障 × L0–L4 × 3 种子 × 3 模式 = 189 次（easy）"],
  ["结果", "easy：183 次正常 + 6 次初始化失败；hard 只完成 96/189 次"],
], 0.6, 1.8, 12.1, [1.8, 10.3], 13);
d.bullets(s, [
  "有效帧 = 跟踪状态 OK 且至少跟踪到 1 个地图点；覆盖率 = 有效帧 / 全部帧",
  "在线 ATE 按 (map_id, frame_epoch) 分段对齐，同时报告段数和失跟次数；final ATE 只作副指标",
  "恢复 = 连续 ≥3 帧有效且误差 ≤ 0.5 m；覆盖率 < 0.1 的运行不计入 ATE",
], 0.6, 4.6, 12.1, 2.2, 14);
s.addNotes("ORB-SLAM3 在重置地图、IMU 初始化、回环时会换世界坐标系，所以在线轨迹必须分段对齐。段数越多 ATE 越偏小，后面所有 ATE 都要结合段数看。");

pres.addSection({ title: "结果" });
s = d.content("结果", "谁先失效", "RGB-D+IMU 几乎不受影响；两种单目模式大多在第 3 档跌破 0.5");
d.image(s, fig("03_coverage_vs_level_easy.png"), 2475 / 600, 0.6, 1.75, 12.1, 2.95, "B coverage easy");
d.table(s, [
  ["覆盖率首次 < 0.5", "高斯模糊", "运动模糊", "低照度", "遮挡", "纹理抹除"],
  ["MONOCULAR", "L3（0.149）", "L3（0.440）", "未跌破（0.550）", "L3（0.469）", "L3（0.480）"],
  ["IMU_MONOCULAR", "L2（0.442）", "L3（0.487）", "未跌破（0.541）", "L3（0.273）", "L3（0.294）"],
  ["IMU_RGBD", "L4（0.032）", "未跌破（0.863）", "未跌破（0.862）", "未跌破（0.925）", "未跌破（0.833）"],
], 0.6, 4.85, 12.1, [2.1, 2.0, 2.0, 2.0, 2.0, 2.0], 12);
d.caption(s, "easy，3 个种子均值；“未跌破”括号内为 L4 的值。来源：results/B/visual_stress.csv", 0.6, 6.7, 12.1);
s.addNotes("横轴是强度，L0 是无扰动。单目+IMU 在高斯模糊下最早失效；RGB-D+IMU 只有在最强高斯模糊下才失效。");

s = d.content("结果", "加 IMU 的单目并不总是更稳", "强遮挡、强纹理抹除下，单目+IMU 更容易丢，种子间差异也大");
d.image(s, fig("04_lost_events_vs_level_easy.png"), 2475 / 600, 0.6, 1.75, 12.1, 2.95, "B lost events easy");
d.card(s, 0.6, 4.9, 3.9, 1.75, "0.142 vs 0.250", "纹理抹除 L4 覆盖率：单目+IMU 对纯单目", C.accent1);
d.card(s, 4.72, 4.9, 3.9, 1.75, "45.3 vs 21.0", "遮挡 L4 失跟次数：单目+IMU 对纯单目", C.accent1);
d.card(s, 8.84, 4.9, 3.9, 1.75, "σ = 0.32", "遮挡 L2 单目+IMU 覆盖率的种子间标准差", C.accent4);
s.addNotes("IMU 未初始化完成时，ORB-SLAM3 一失跟就清空地图重来；图像差时重置更频繁，所以单目+IMU 在强遮挡和强纹理抹除下反而更差。");

s = d.content("结果", "ATE 要和段数、覆盖率一起读", "强扰动下残存的短片段会让 ATE 看起来很小");
d.image(s, fig("05_ate_vs_level_easy.png"), 2475 / 600, 0.6, 1.75, 12.1, 2.95, "B ATE easy");
d.bullets(s, [
  "纹理抹除 L4 纯单目：ATE 0.076 m，但只有 12% 的帧参与对齐，平均 113 段",
  "覆盖率 < 0.1 的点不画 ATE（如高斯模糊 L4 的 RGB-D+IMU，覆盖率 0.032）",
  "回环会放大 final ATE：遮挡 L4 RGB-D+IMU 的 final ATE 2.736 m，在线 ATE 0.060 m",
], 0.6, 4.85, 12.1, 2.0, 14);
s.addNotes("ATE 小不代表跟得好。每段单独对齐，段越短误差越小。");

s = d.content("结果", "典型失败：最强高斯模糊", "单目两种模式完全无法初始化；RGB-D+IMU 能初始化，但只剩 3% 的有效帧");
d.image(s, fig("06_case_gaussian_blur_L4_IMU_MONOCULAR.png"), 1585 / 926, 0.6, 1.75, 6.0, 4.6, "case IMU_MONOCULAR");
d.image(s, fig("07_case_gaussian_blur_L4_IMU_RGBD.png"), 1504 / 926, 6.7, 1.75, 6.0, 4.6, "case IMU_RGBD");
d.caption(s, "左：单目+IMU（6 次初始化失败中的一次）　右：RGB-D+IMU（10 段，覆盖率 0.032）", 0.6, 6.45, 12.1);
s.addNotes("easy 上 6 次初始化失败全部在高斯模糊 L4，纯单目和单目+IMU 各 3 个种子。");

s = d.content("结果", "hard 序列：只作现象描述", "无扰动基线本身已大量失败，退化效果分不开，只跑了 96/189 次");
d.image(s, fig("08_coverage_vs_level_hard.png"), 2475 / 600, 0.6, 1.75, 12.1, 2.95, "B coverage hard");
d.bullets(s, [
  "无扰动覆盖率只有 0.450 / 0.302 / 0.624（MONOCULAR / IMU_MONOCULAR / IMU_RGBD），平均 86.7 / 98.7 / 154.3 段",
  "原因：hard 转得快，IMU 初始化需要 10 个关键帧，未初始化完一失跟就清空地图",
  "已完成：高斯模糊、运动模糊 L1–L4；低照度 L1–L2；遮挡 L2 的 1 个种子；纹理抹除未跑",
], 0.6, 4.85, 12.1, 2.0, 14);
s.addNotes("hard 不在验收线内，只用来说明快速运动下 ORB-SLAM3 本身的限制。");

pres.addSection({ title: "总结" });
s = d.content("总结", "局限", "");
d.bullets(s, [
  "只有一个场景（Office/P004）；趋势只在 easy 上成立",
  "退化是对录好的图像做的离线处理；纹理抹除不等于真实无纹理几何",
  "在线 ATE 按段对齐，段越多越偏小；强扰动下单目可能存在未检出的坐标系缓慢漂移",
  "ORB-SLAM3 是多线程的，重复运行有差异；每个设置只有 3 个种子",
  "hard 序列只完成 96/189 次",
], 0.6, 1.3, 12.1, 5.5, 16);
s.addNotes("如实交代局限。");

d.cover("总结", "图像变差时，深度 + IMU 最稳",
  [{ text: "单目系统在第 3 档左右失效；加 IMU 不保证更稳", options: { breakLine: true } }, { text: "代码、配置与结果：experiment/b-visual 分支" }],
  "总结并给出复现入口。");

d.save(OUT, SKILL).catch(e => { console.error(e); process.exit(1); });
