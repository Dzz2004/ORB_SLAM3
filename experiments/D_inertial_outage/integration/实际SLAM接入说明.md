# D实际SLAM初始化：已在P000运行验证

已自行运行ArchVizTinyHouseDay/easy/P000的RGB-D+IMU基线，不再依赖A提供状态。
实际输入与原始日志位于`results/D/slam_easy_P000/`：

- `slam_states.csv`：位姿、速度、两类偏置、有效/初始化标记、地图及世界坐标版本。
- `slam_metadata.json`：数据、坐标、单位、重力和源码版本。
- `baseline_run.log`：实际运行日志。

## 已完成的运行

官方ORB-SLAM3源码基于本仓库`4861c07`，无界面运行器复用
[B/C支持分支132f83c](https://github.com/Dzz2004/ORB_SLAM3/tree/132f83c)，D增加完整状态导出。
隔离源码在仓库同级`data_D/baseline/project/`，未修改仓库公共SLAM源文件。

本机环境：WSL Ubuntu20.04；GCC11.4、CMake3.30、OpenCV4.10、Eigen3.4、Pangolin0.6。
主库、g2o与运行器统一C++11及`-O3 -march=native`，运行器同时使用`-DNDEBUG`。

P000：640×640、10Hz图像/深度，100Hz官方原始无噪声IMU，跳过起始0.2秒样条边缘。
标定及深度缩放见`calib.yaml`。深度由官方四通道浮点PNG转为毫米uint16，超过65.535米设无效。
数据来自[官方镜像](https://huggingface.co/datasets/theairlabcmu/tartanair2/tree/main/ArchVizTinyHouseDay/Data_easy)，
只取P000的`image_lcam_front`、`depth_lcam_front`、`imu`。

本机已编译并准备好数据，重跑基线使用新的输出名：

```powershell
wsl -d Ubuntu-20.04 -- bash /mnt/c/Users/27427/Desktop/虚拟现实技术/data_D/baseline/run_p000.sh p000_rgbdi_02 IMU_RGBD
```

## 换电脑时接入导出源码

1. 使用上述固定提交的ORB-SLAM3源码和B/C支持分支中的编译、数据转换脚本。
2. 在ORB-SLAM3根目录执行`git apply /绝对路径/ORB_SLAM3_D.patch`。
3. 用本目录`run_orbslam.cc`替换B/C运行器，采用本目录`calib.yaml`。
4. 按上述版本配置依赖。第三方库CMake传`-DCMAKE_CXX_STANDARD=11`，运行器添加`-DNDEBUG`。
5. 用B/C统一格式准备`manifest.csv`、`rgb/`、`depth/`、`imu.csv`，运行：

```text
run_orbslam IMU_RGBD ORBvoc.txt calib.yaml 输入序列目录 新输出目录 --rate 1 --start 0.2
```

`GetDImuState`只在跟踪正常、IMU已初始化且Frame有位姿/速度时返回有效数据。
调用`GetImuPosition/GetImuRotation/GetVelocity/mImuBias`，机体轴为原始IMU轴；
世界为SLAM世界，重力`[0,0,-9.81]`。普通TUM轨迹缺速度和偏置，不能替代状态CSV。

## D正式实验与评分

执行实验README中的完整命令，生成理想上界、实际SLAM实验及统一结果。
正式分支只从CSV读取初始位姿、速度、偏置；失效后只积分IMU。
仅选失效前同地图、同世界坐标版本的最近可信帧，不读取未来状态。

预测完成后，用失效前历史位姿计算固定旋转和平移做离线评分，不拟合尺度。
旋转由历史姿态确定，不用两个位置点反推旋转；本轮使用2～11帧历史（0.1～1秒），
每个窗口的历史范围均记入`metrics.json`。失效后真值不参与配准或纠偏。

## 本轮范围

单目+IMU反复重置、未完成惯性初始化，没有用于正式预测。
RGB-D+IMU取得42帧有效状态，首次初始化7.4秒；取7.5～8.4秒的10个运动起点。
每个起点3档噪声、3个种子，评价0.5/1/2/3秒，共360条记录。
窗口有重叠；实际初始化只验证easy/P000，hard/P000仍为理想上界。
