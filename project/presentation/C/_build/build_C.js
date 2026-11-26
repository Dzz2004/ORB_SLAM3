// C depth-degradation deck. Usage: node build_C.js <presentation/C dir> <out.pptx> <pptx skill dir>
// Every number comes from presentation/C/slides.md and 实现说明.md (checked against results/C/depth_ablation.csv).
const path = require("path");
const { makeDeck } = require("./deck_lib");
const [, , DIR, OUT, SKILL] = process.argv;
const fig = name => path.join(DIR, "figures", name);

const d = makeDeck("C：深度退化下的 ORB-SLAM3", "ORB-SLAM3 · C 深度退化");
const { pres, C } = d;

pres.addSection({ title: "开场" });
d.cover("开场", "C：深度变差时，系统怎样退化",
  "无效像素 · 测距噪声 · 整帧中断 · 完全没有深度　|　TartanAir V2 Office/P004",
  "C 部分研究：深度给融合系统带来了什么；深度逐渐失效时，RGB-D 和 RGB-D+IMU 怎样退化，会不会自动退回单目惯性。");

let s = d.content("开场", "结论", "RGB-D+IMU 对深度退化很不敏感；但完全没有深度时，ORB-SLAM3 不会自动退回单目惯性");
d.card(s, 0.6, 1.9, 3.9, 2.0, "0.028 m", "80% 无效深度下 RGB-D+IMU 的在线 ATE，覆盖率 0.987（无扰动 0.036 m）", C.accent2);
d.card(s, 4.72, 1.9, 3.9, 2.0, "1.000", "0.5–2 s 整帧深度中断期间的有效帧比例（easy 18/18 次运行）", C.accent2);
d.card(s, 8.84, 1.9, 3.9, 2.0, "12 / 12", "整条没有深度时初始化失败的运行数", C.accent1);
d.bullets(s, [
  "测距噪声 r ≤ 5% 对 RGB-D+IMU 没有可分辨影响；r = 10% 时 ATE 升高 62%",
  "纯 RGB-D 在 80% 无效深度时 ATE“变小”是假象：轨迹被切成约 99 段",
  "162 次运行中有 2 次 ORB-SLAM3 自身崩溃（纯 RGB-D，Sophus SO3::exp 得到 NaN）",
], 0.6, 4.25, 12.1, 2.4, 16);
d.caption(s, "easy 序列，3 个种子均值。来源：results/C/depth_ablation.csv", 0.6, 6.7, 12.1);
s.addNotes("先给结论：深度 + IMU 很稳；完全没有深度时系统起不来；ATE 必须结合段数读。");

pres.addSection({ title: "实验设置" });
s = d.content("实验设置", "深度相机会怎样失效", "成片无效、噪声随距离增大、短时整帧丢失，以及完全没有深度");
d.image(s, fig("01_depth_mosaic_easy.png"), 1990 / 1314, 0.6, 1.75, 7.4, 4.85, "C mosaic easy");
d.bullets(s, [
  "无效像素：在原有基础上再加 0 / 20 / 50 / 80%",
  "测距噪声：σ = r·z，r = 1 / 2 / 5 / 10%",
  "整帧中断：从 10 s 起 0.5 / 1 / 2 s",
  "整条无深度：depth_off",
  "原始数据里超过 65.5 m 的深度本来就是 0（easy 平均 4.2%、hard 2.6% 的像素）",
], 8.3, 1.85, 4.4, 4.8, 15);
d.caption(s, "图：figures/01_depth_mosaic_easy.png（黑色 = 无效深度）", 0.6, 6.7, 7.4);
s.addNotes("深度给 SLAM 直接提供尺度。真实深度相机常见三种失效：成片无效、测距噪声随距离增大、短时整帧丢失。");

s = d.content("实验设置", "实验矩阵与评测口径", "同一序列、同一故障表、同一评测口径，每个设置 3 个种子");
d.table(s, [
  ["项目", "设置"],
  ["序列", "TartanAir V2 Office/P004：easy 1220 帧（122 s）、hard 817 帧（82 s），10 Hz，640×640"],
  ["模式", "RGBD、IMU_RGBD；无扰动 IMU_MONOCULAR 作为“RGB+IMU”对照"],
  ["规模", "2 条序列 × 27 种设置 × 3 种子 = 162 次"],
  ["结果", "148 次正常 + 12 次初始化失败（全部是无深度）+ 2 次崩溃"],
], 0.6, 1.8, 12.1, [1.8, 10.3], 13);
d.bullets(s, [
  "有效帧 = 跟踪状态 OK 且至少跟踪到 1 个地图点（否则无深度时空地图会被误算成“跟踪成功”）",
  "在线 ATE 按 (map_id, frame_epoch) 分段对齐，段数和覆盖率一起报告；final ATE 只作副指标，并标明是否回环",
  "恢复 = 连续 ≥3 帧有效且误差 ≤ 0.5 m；未恢复的事件单独计数",
], 0.6, 4.6, 12.1, 2.2, 14);
s.addNotes("强调有效帧的定义：没有深度时，ORB-SLAM3 每帧都报告跟踪 OK，但地图里一个点都没有。");

pres.addSection({ title: "结果" });
s = d.content("结果", "无效深度：RGB-D+IMU 几乎不受影响", "50% 以内两种模式的 ATE 基本不变；80% 时 RGB-D 的“更准”是分段带来的假象");
d.image(s, fig("02_invalid_ratio_easy.png"), 1125 / 1380, 0.6, 1.7, 4.3, 5.3, "C invalid easy");
d.table(s, [
  ["easy，在线 ATE（m）", "无扰动", "P50", "P80"],
  ["IMU_RGBD", "0.036", "0.029", "0.028"],
  ["RGBD", "0.097", "0.094", "0.026（98.7 段）"],
  ["RGB+IMU 对照", "0.040", "—", "—"],
], 5.2, 1.85, 7.5, [2.5, 1.5, 1.5, 2.0], 13);
d.bullets(s, [
  "RGB-D 在 P80 时段数从 16.7 涨到 98.7，频繁重新初始化；final ATE 为 0.083 m，在线 ATE 不可比",
  "RGB-D+IMU 覆盖率始终 ≈ 0.99，段数只从 12.3 增到 23.0",
  "hard 同理：P80 的 RGB-D+IMU 在线 ATE 0.020 m，但 186.7 段里只有 24.7 段参与对齐",
], 5.2, 4.2, 7.5, 2.6, 14);
s.addNotes("看第三个子图：蓝线 RGB-D 的段数在 80% 时暴涨。ATE 降低不是更准，是轨迹被切碎了。");

s = d.content("结果", "测距噪声：10% 才看得出影响", "r ≤ 5% 对 RGB-D+IMU 没有可分辨影响；r = 10% 时 ATE 升高 62%");
d.image(s, fig("03_noise_easy.png"), 1125 / 1380, 0.6, 1.7, 4.3, 5.3, "C noise easy");
d.card(s, 5.2, 1.9, 3.6, 1.9, "0.030–0.034 m", "RGB-D+IMU 在 r = 1–5% 时的在线 ATE", C.accent2);
d.card(s, 9.1, 1.9, 3.6, 1.9, "0.058 m", "r = 10% 时（无扰动 0.036 m）", C.accent1);
d.bullets(s, [
  "RGB-D 的 ATE 随 r 增大反而下降，同样是分段伪影（段数 13 → 86）",
  "RGB-D+IMU 在 r = 10% 时覆盖率仍 ≈ 0.995",
  "噪声模型是相对噪声 σ = r·z，近处误差小、远处误差大",
], 5.2, 4.15, 7.5, 2.6, 14);
s.addNotes("RGB-D+IMU 对测距噪声很不敏感，只有到 10% 才有明显退化。");

s = d.content("结果", "深度中断：RGB 一路接住", "0.5 / 1 / 2 s 整帧深度中断期间，跟踪一次也没有断");
d.image(s, fig("04_outage_easy.png"), 1125 / 1140, 0.6, 1.7, 5.2, 5.2, "C outage easy");
d.bullets(s, [
  "easy 18/18 次运行在中断窗口内有效帧比例都是 1.000，窗口后恢复时间 0.0 s",
  "RGB-D 整次运行有 1–2 次失跟（单次 0–4 次），但全部在中断窗口之外，最早 28.0 s，与中断无关",
  "RGB-D+IMU 没有失跟",
  "hard：窗口内有效帧比例 RGB-D 0.83–1.00、RGB-D+IMU 0.12–0.60；但 hard 无扰动时每次运行就有 148–152 次失跟，分不开中断的影响",
], 6.1, 1.85, 6.6, 5.0, 14);
s.addNotes("中断期间深度没了，但 RGB 图像还在，ORB-SLAM3 照样能跟踪。");

s = d.content("结果", "完全没有深度：系统起不来", "ORB-SLAM3 不会自动退回单目惯性，而是每帧建空地图、下一帧就丢");
d.image(s, fig("05_status_easy.png"), 1125 / 1488, 0.6, 1.7, 4.0, 5.3, "C status easy");
d.card(s, 4.9, 1.9, 3.8, 1.9, "12 / 12", "整条无深度时初始化失败（easy + hard，两种模式，3 个种子）", C.accent1);
d.card(s, 8.9, 1.9, 3.8, 1.9, "609 帧", "easy RGB-D：报告 OK 但跟踪点数全为 0 的帧", C.accent4);
d.bullets(s, [
  "每帧都“初始化成功”一张 0 个点的空地图，下一帧就丢，交替 609 帧 OK / 609 帧 LOST",
  "关键帧永远攒不够，初始化完成不了；覆盖率为 0",
  "2 次崩溃：纯 RGB-D、无效像素、种子 2，都在序列末尾，Sophus SO3::exp 得到 NaN；RGB-D+IMU 没有崩溃",
], 4.9, 4.15, 7.8, 2.6, 14);
s.addNotes("这回答了方案里的问题：RGB-D+IMU 面对无深度时，是否会无缝切换成单目惯性？实测不会。");

pres.addSection({ title: "总结" });
s = d.content("总结", "局限", "");
d.bullets(s, [
  "只有一个场景（Office/P004）的两条序列；hard 序列无扰动时就大量失败，趋势以 easy 为准",
  "退化是对录好的深度图做的离线处理，没有模拟真实 ToF 的成像过程",
  "在线 ATE 按段对齐，段越多越偏小；IMU 模式回环后 final ATE 会被拉到 6–8 m",
  "ORB-SLAM3 是多线程的，重复运行有差异；每个设置只有 3 个种子",
  "IMU 由真值轨迹生成后加噪声，比真实 IMU 理想",
], 0.6, 1.3, 12.1, 5.5, 16);
s.addNotes("如实交代局限。");

d.cover("总结", "深度变差时，深度 + IMU 仍然稳",
  [{ text: "但没有深度时系统起不来；ATE 要结合段数读", options: { breakLine: true } }, { text: "代码、配置与结果：experiment/c-depth 分支" }],
  "总结并给出复现入口。");

d.save(OUT, SKILL).catch(e => { console.error(e); process.exit(1); });
