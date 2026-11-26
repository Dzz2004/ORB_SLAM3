# 环境与数据准备

本目录（`project/`）是 B/C 实验的工作根目录，下文记作 `ROOT`。以下命令都在 `ROOT` 下执行，环境和数据都建在 `ROOT` 里，不需要 root 权限，也不需要 GPU。原始数据、扰动序列、编译产物和环境目录体积较大，没有放进 git（见 `.gitignore`），按下面的步骤重新生成。

## 1. ORB-SLAM3 编译环境（`env/`）

用 conda-forge 的编译器和依赖建一个独立环境，再从源码编译 Pangolin v0.6 和 ORB-SLAM3：

```bash
mamba create -y -p $PWD/env -c conda-forge --override-channels \
  python=3.10 cmake make pkg-config gxx_linux-64=11 gcc_linux-64=11 libopencv=4.10 eigen=3.4 \
  libboost-devel openssl 'numpy<2' glew libgl-devel libegl-devel libglu xorg-libx11 pip
env/bin/python -m pip install evo 'opencv-python-headless<4.11' 'numpy<2'

git clone https://github.com/Dzz2004/ORB_SLAM3 src/ORB_SLAM3        # 实验用的是 master 4861c078
git clone --depth 1 -b v0.6 https://github.com/stevenlovegrove/Pangolin src/Pangolin
cmake -S src/Pangolin -B src/Pangolin/build -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PWD/env -DCMAKE_PREFIX_PATH=$PWD/env -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
  -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_VIDEO=OFF -DBUILD_TESTS=OFF \
  -DCMAKE_CXX_FLAGS="-include limits -include cstdint"
cmake --build src/Pangolin/build -j8 && cmake --install src/Pangolin/build

scripts/build_orbslam3.sh 8   # DBoW2、g2o、Sophus、解压词典、ORB-SLAM3
scripts/build_runner.sh       # scripts/run_orbslam.cc -> bin/run_orbslam（不启动 Pangolin 窗口）
```

ORB-SLAM3 源码没有改动。编译时遇到的问题和解决办法：

- conda-forge 没有 Pangolin，所以从源码编译，并关掉 video、python、examples。
- CMake 4 不再接受 `cmake_minimum_required(VERSION < 3.5)`，统一加 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`。
- 链接 `bin/run_orbslam` 时需要 `-rpath-link` 和 `-lGLEW`；它必须和 ORB-SLAM3 库用同样的 `-O3 -march=native` 编译，否则 Eigen 内存对齐不一致，运行时会崩溃。`build_runner.sh` 已经处理好。

## 2. 数据与评测环境（`env_data/`）

下载、格式转换、扰动生成、评测、出图都在一个轻量 venv 里运行。tartanairpy 依赖 torch 和 cupy，这里不装，下载脚本直接访问它使用的同一个公开数据源。

```bash
python3.10 -m venv env_data
env_data/bin/pip install numpy scipy opencv-python-headless pyyaml matplotlib evo boto3
```

## 3. 数据：TartanAir V2 Office/P004（`data/seqs/`）

```bash
env_data/bin/python scripts/download_tartanair.py --env Office --diff easy hard --traj P004 --out data/raw
for d in easy hard; do
  env_data/bin/python scripts/convert_tartanair.py --raw data/raw --env Office --diff $d --traj P004 \
      --out data/seqs/office_${d}_p004 --noisy-imu-seed 0
done
env_data/bin/python scripts/make_noisy_imu.py data/seqs/office_easy_p004 data/seqs/office_hard_p004 --seeds 0 1 2
```

每条序列转换后的统一格式：

| 文件 | 内容 |
|---|---|
| `manifest.csv` | `timestamp,rgb,depth,gt_index`，时间戳单位秒 |
| `rgb/*.png` | 8 位 BGR，640×640，10 Hz |
| `depth/*.png` | uint16，值 = 米 × 1000，0 = 无效（超过 65.535 m 的像素本来就是 0） |
| `imu.csv` | `timestamp,gx,gy,gz,ax,ay,az`，100 Hz，SI 单位，无噪声 |
| `imu_noisy_s<种子>.csv` | 按 `calib.yaml` 的噪声参数加了白噪声和偏置随机游走，正式实验都用这个 |
| `trajectory_gt.tum` | 真值 `T_w_c`，相机光学坐标系，世界系为 TartanAir NED |
| `calib.yaml` | ORB-SLAM3 配置，四种模式通用（fx = fy = cx = cy = 320，无畸变） |

| 序列 | 帧数 | 时长 |
|---|---|---|
| `office_easy_p004` | 1220 | 121.9 s |
| `office_hard_p004` | 817 | 81.6 s |

## 4. 运行与评测入口

- `scripts/run_orbslam.sh MODE SEQ_DIR OUT_DIR`：MODE 可选 `MONOCULAR`、`IMU_MONOCULAR`、`RGBD`、`IMU_RGBD`。用环境变量 `IMU_SEED=s` 选择 `imu_noisy_s<s>.csv`。默认按真实 10 Hz 回放（`--rate 1`），并跳过开头 0.2 s 以避开 IMU 样条端点效应。每次运行输出 `tracking.csv`（逐帧状态、跟踪点数、地图编号 `map_id`、坐标系版本号 `frame_epoch`）、`trajectory_online.tum`（在线轨迹，主指标）、`trajectory_est.tum`（ORB-SLAM3 保存的优化后轨迹，副指标）和 `run.log`。
- `scripts/run_matrix.py <矩阵 yaml> --workers N`：按矩阵批量生成扰动序列、运行、评测，可以断点续跑；`--reevaluate` 只重新评测；`--cleanup` 用完删除扰动序列。
- `scripts/evaluate.py`：ATE/RPE、覆盖率、失跟事件、恢复时间，输出 `metrics.json`。`scripts/tests/test_evaluate.py` 是它的自测。
- `scripts/aggregate.py`：把所有 `metrics.json` 汇总成逐次运行和分组两张 CSV。

**并发**：ORB-SLAM3 是多线程的，CPU 不够时跟踪会跟不上 10 Hz，结果就失真。实验所用容器只有 4 核 CPU 配额，我们实测同时运行超过 4 个 ORB-SLAM3 后覆盖率明显下降，所以所有正式实验 `--workers` 合计不超过 4。换机器时请先按核数重新确定。
