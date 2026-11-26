# A 部分基线数据 PPT 调整与讲解文本

## 建议调整

把原来单张“Easy 基线结果”和“Hard 基线结果”拆成四张页面：

1. Easy 三 case 总览：`easy_three_cases_bars.png`
2. Easy 关键现象：为什么 P000/P001/P002 的排序不同
3. Hard 三 case 总览：`hard_three_cases_bars.png`
4. Hard 失败解释：初始化、失跟和地图连续性

图中必须保留以下口径：

- 柱高是均值，误差棒是有效 seed 的样本标准差。
- 柱顶 `n/3` 是有效 seed 数，不是总运行数。
- `N/A` 表示该指标无法计算，不是 0。
- 覆盖率和耗时使用全部 3 个 seed。
- ATE/RPE 当前使用优化关键帧主轨迹，只用于基线诊断；最终公平精度结论仍应按官方最大地图全帧轨迹复核。
- 耗时只统计 ORB-SLAM3 `Track()` 调用，不包含图像读取和深度解码。

## 第 1 页：Easy 三 case 总览

### 页面标题

Easy：融合提高了多数 case 的可跟踪性，但精度收益并不一致

### 页面内容

放置 `easy_three_cases_bars.png`。重点圈出：

- Easy P000：RGB+IMU 的 ATE 为 `0.090 m`，但只有 `2/3` 个 seed 能计算 1 m RPE。
- Easy P001：RGB+IMU 的 ATE 为 `0.300 ± 0.254 m`，RPE 在 `0/3` 个 seed 中可计算。
- Easy P002：RGB+IMU 和 RGB-D+IMU 均优于 RGB，三个模式的覆盖率都约为 98%–99%。

### 口播讲稿

先看 Easy。这里不是简单地说“融合一定更准”，而是同时看精度、有效 seed 和覆盖率。

P000 中 RGB+IMU 的平均 ATE 最低，但它只有两个 seed 能给出 1 米间隔的 RPE，覆盖率为
73.3%，P95 跟踪耗时为 100.3 ms。RGB-D+IMU 的 ATE 稍高，但覆盖率保持在 98%，P95
耗时约为 32.7 ms。因此，RGB+IMU 是“精度条件最好但可用性代价较大”。

P001 更能说明指标缺失不能当成零。RGB+IMU 三个 seed 都能得到 ATE，但关键帧轨迹很稀，
三个 seed 都没有足够的 1 米 RPE 配对。这部分必须展示 `N/A 0/3`，不能用零代替。

P002 中 RGB+IMU 和 RGB-D+IMU 的 ATE 分别为 0.326 m 和 0.281 m，均低于 RGB 的
1.103 m，而且覆盖率均约为 99%。因此 Easy 的三个 case 已经说明：融合收益存在明显的
场景依赖。

### 图注

Easy P000/P001/P002 的 ATE、1 m RPE、有效位姿覆盖率和 `Track()` 调用耗时。柱高为均值，
误差棒为有效 seed 的样本标准差，柱顶标注有效 seed 数；`N/A` 表示指标不可计算。

### Alt text

三行四列柱状图，分别展示 Easy P000、P001、P002 下 RGB、RGB+IMU、RGB-D+IMU 的 ATE、
RPE、覆盖率和 P95 耗时。P000 和 P001 的 RGB+IMU RPE 存在缺失，P002 两种融合模式的
ATE 均明显低于 RGB。

## 第 2 页：Easy 结果如何解释

### 页面标题

Easy 的排序变化来自初始化时间、轨迹密度和跟踪覆盖率

### 页面内容

使用三条要点，不重复整张大图：

- P000：RGB+IMU 的地图有效段开始较晚，覆盖率和 RPE 有效 seed 都下降。
- P001：RGB+IMU 的 ATE 有值，但关键帧过稀，RPE 全部不可用。
- P002：轨迹较稳定，两种融合模式都优于 RGB。

### 口播讲稿

这三条 case 不能用一句“融合最好”概括。P000 和 P001 都出现了指标可用性问题：系统中
有逐帧 tracking 位姿，但最终可用于连续评价的优化关键帧轨迹不足。P002 的轨迹更稳定，
所以融合优势能完整显现。

同时，当前 ATE 使用的是关键帧主轨迹，RGB 与融合模式实际参与评测的帧数不同。因此这一页
应定位为“基线运行现象”，而不是最终公平精度排名。正式结论应补充官方最大地图全帧轨迹
复算和共同的评价时间区间。

## 第 3 页：Hard 三 case 总览

### 页面标题

Hard：主要问题从精度转向初始化、失跟和地图连续性

### 页面内容

放置 `hard_three_cases_bars.png`。重点圈出：

- Hard P000：RGB 的 ATE 为 `0.588 m`、覆盖率 `91%`；RGB+IMU 仅 `1/3` 个 seed 有 ATE，
  覆盖率 `24.7%`；RGB-D+IMU 为 `0/3`。
- Hard P001：三个模式均没有可作连续评测的 ATE；覆盖率 RGB `50.3%`、
  RGB+IMU `28.0%`、RGB-D+IMU `60.0%`。
- Hard P002：RGB 仅 `1/3` 个 seed 有 ATE；两个惯性模式均为 `0/3`。

### 口播讲稿

Hard 的数据首先要看覆盖率和有效 seed，不能只看 ATE。

P000 中 RGB 仍有 91% 覆盖和 0.588 m 的 ATE。RGB+IMU 只有一个 seed 得到 ATE，平均
覆盖率降到 24.7%。RGB-D+IMU 虽然保留了 58% 的逐帧跟踪覆盖，但没有形成三个 seed
都可用的关键帧主轨迹，所以 ATE 记为 N/A。

P001 中三个模式都没有连续 ATE。覆盖率分别是 RGB 50.3%、RGB+IMU 28.0% 和
RGB-D+IMU 60.0%。这里的低覆盖率与日志中的 IMU 未初始化、局部地图跟踪失败和 active
map reset 一致。

P002 中 RGB 只有一个 seed 可用，ATE 为 4.877 m；两个惯性模式没有可用 ATE。
因此 Hard 的主要结论是基线发生了真实的可用性退化，而不是某个模式在所有场景中都更准确。

### 图注

Hard P000/P001/P002 的三模式基线结果。`N/A` 表示未形成可评测的单地图连续主轨迹；
覆盖率和耗时仍基于全部三个 seed 的逐帧 tracking 日志。

### Alt text

三行四列 Hard 柱状图。RGB 在 P000 和 P002 有部分 ATE，RGB+IMU 仅 P000 有一个有效
seed；P001 三个模式均无 ATE。覆盖率普遍低，RGB+IMU 的 P95 耗时最高。

## 第 4 页：Hard 失败机制

### 页面标题

Hard 失败不是“统一调参没调好”，而是 ORB-SLAM3 初始化条件未被满足

### 页面内容

建议使用一个简化流程：

```text
视觉关键帧不足
  -> 单目惯性 10 KF / 2 s 初始化门槛未满足
  -> active map reset
  -> 新地图没有共同世界坐标系
  -> 全窗口 ATE/RPE 不可定义
```

### 口播讲稿

ORB-SLAM3 的单目惯性初始化要求至少 10 个关键帧，并覆盖约 2 秒。Hard P000、P002
以及 P001 的多个惯性 run 只有 0 到 3 个关键帧，因此初始化无法完成，日志会反复出现
“IMU is not or recently initialized” 和“Reseting active map”。

地图重置后，新地图有独立的世界坐标。把不同 map 的位姿直接拼接后计算 ATE/RPE，
数值没有物理意义。所以这里保留逐帧 tracking 覆盖率和逐 map 诊断图，但连续 ATE/RPE
记为 N/A。

## 第 5 页：结论页素材

### 页面标题

基线结论：融合改善并非普遍成立，必须同时报告精度与可用性

### 页面内容

- Easy：P002 支持融合收益；P000/P001 显示精度、覆盖率和指标可用性之间存在权衡。
- Hard：失跟、IMU 初始化和地图重置比单纯 ATE 更关键。
- 指标：所有图表同时展示有效 seed、覆盖率；缺失值不按零处理。
- 方法边界：当前关键帧主轨迹用于基线诊断，全帧轨迹复算后再形成最终公平排名。

### 口播讲稿

A 部分的基线结论是：融合并不保证每条序列都更准确。Easy P002 支持融合改善精度，
但 P000 和 P001 暴露了覆盖率和指标可用性的代价。Hard 则主要暴露 IMU 初始化和地图
连续性失败。

因此汇报时应同时展示 ATE、RPE、覆盖率和有效 seed 数。最终的公平排名还需要把主评测
轨迹统一到官方最大地图全帧轨迹，并在所有模式共有的有效时间区间上复算。

## 图表文件

```text
presentation/A/figures/data/
├─ easy_P000_bars.png
├─ easy_P001_bars.png
├─ easy_P002_bars.png
├─ easy_three_cases_bars.png
├─ hard_P000_bars.png
├─ hard_P001_bars.png
├─ hard_P002_bars.png
├─ hard_three_cases_bars.png
├─ *.svg
└─ figure_manifest.json
```
