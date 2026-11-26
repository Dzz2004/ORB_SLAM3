# D：视觉与深度失效后的IMU短时预测

从实际SLAM最后可信的位姿、速度和IMU偏置出发，只用IMU预测0.5～3秒。

## 关键文件

| 文件 | 用途 |
|---|---|
| `imu_dead_reckoning.py` | IMU中点积分算法，输出位置、速度和朝向 |
| `run_experiment.py` | 唯一运行入口：测试、全部实验、统计和绘图 |
| `download_data.ps1` | 下载并校验官方easy/hard IMU数据 |
| `requirements.txt` | Python依赖 |
| `tests/` | 积分器与SLAM状态读取测试 |
| `integration/slam_state.py` | 读取实际状态，拒绝未来帧、旧地图和过期状态 |
| `integration/ORB_SLAM3_D.patch` | 从ORB-SLAM3导出真实位姿、速度和偏置的源码补丁 |
| `integration/run_orbslam.cc` | 已编译验证的无界面基线运行器 |
| `integration/calib.yaml` | P000的相机、IMU、深度参数 |

## 运行

在仓库根目录执行，本机环境和数据已就绪：

```powershell
.\.venv-d\Scripts\python.exe -B experiments/D_inertial_outage/run_experiment.py
```

换电脑先安装依赖并下载数据：

```powershell
python -m venv .venv-d
.\.venv-d\Scripts\python.exe -m pip install -r experiments/D_inertial_outage/requirements.txt
powershell -ExecutionPolicy Bypass -File experiments/D_inertial_outage/download_data.ps1
powershell -ExecutionPolicy Bypass -File experiments/D_inertial_outage/download_data.ps1 -Difficulty hard
```

真实SLAM初始状态已随结果提交，重跑预测不需要重新编译SLAM。
如需重跑基线，见[integration/实际SLAM接入说明.md](integration/实际SLAM接入说明.md)。

## 输出

| 位置 | 内容 |
|---|---|
| `results/D/outage_error.csv` | 1080条逐实验记录：实际360条、理想720条 |
| `results/D/summary.csv` | 各噪声、时长的平移/旋转误差中位数、P90、阈值通过率 |
| `results/D/metrics.json` | 配置、种子、源码哈希、统计和积分耗时 |
| `results/D/运行日志.txt` | 11项测试及实验日志 |
| `results/D/slam_easy_P000/` | 真实初始状态、基线日志、正式结果、两张图、预测TUM与状态CSV |
| `results/D/easy_P000/`、`hard_P000/` | 理想对照的误差CSV和配置JSON |
| `presentation/D/` | 两页学校模板PPT、两张图、一份简短实现说明；讲稿在PPT备注中 |

## 实验范围

- TartanAir V2的ArchVizTinyHouseDay/P000；IMU为100Hz。
- 每组3档噪声×10个运动起点×3个种子，评价0.5/1/2/3秒。
- 正式结果使用easy/P000的真实RGB-D+IMU状态；easy理想对照使用相同起点/种子/噪声，hard只做理想上界。
- 实际起点7.5～8.4秒，估计速度约0.079～1.34米/秒；窗口重叠，不是10条独立轨迹。
- 正式预测中不使用真值。预测结束后仅用中断前同地图/坐标版本历史做固定旋转和平移配准，不拟合尺度。
- P90曲线反映经验误差范围；状态为`IMU_PREDICTION`，未实现在线概率协方差。
- 数据集重力`[0,0,9.8]`，SLAM重力`[0,0,-9.81]`；四元数顺序xyzw、机体到世界；偏置在机体系。

未将真实数据集、编译目录和虚拟环境放入交付包；GIF是选做，本次未制作。
