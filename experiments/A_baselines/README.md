# A Baseline

## Frozen experiment

- Dataset: TartanAir V2
- Environment: `OldTownSummer`
- Modes: `rgb`, `rgb_imu`, `rgbd_imu`
- Seeds: `0`, `1`, `2`
- Camera: pinhole, 640x640, `fx=fy=320`, `cx=cy=320`, zero distortion
- IMU: 100 Hz, aligned with `lcam_front`

The experiment matrix is:

| Role | Sequence | Frames | Interval | Seeds |
|---|---|---:|---|---|
| Primary | `Data_easy/P000` | 200-299 | 20.0-29.9 s | 0, 1, 2 |
| Holdout | `Data_easy/P002` | 200-299 | 20.0-29.9 s | 0, 1, 2 |
| Hard robustness | `Data_hard/P001` | 100-199 | 10.0-19.9 s | 0, 1, 2 |

The windows are intentionally short because longer hard windows cross ORB-SLAM3 map
resets, which creates discontinuous coordinate frames. The resulting failure rate is
part of the experiment result.

The camera-to-IMU transform is the fixed optical-to-NED axis permutation:

```text
[0 0 1 0]
[1 0 0 0]
[0 1 0 0]
[0 0 0 1]
```

IMU noise and random-walk values use the ORB-SLAM3 EuRoC magnitudes, not a near-zero
placeholder covariance.

The comparison protocol uses the same optimized-keyframe body-frame trajectory as the
primary metric source for every mode. Per-frame and largest-map trajectories are retained
as diagnostic artifacts. ATE/RPE are computed over the full frozen window only when all
valid tracking frames belong to one map; otherwise they are marked unavailable. OpenMP,
OpenBLAS, and MKL are fixed to one thread. RPE uses a 1 m interval. The seed is reapplied
after ORB-SLAM3 construction because the TwoViewReconstruction constructor seeds its own
RNG.

Hard full-window accuracy is not reported as a continuous trajectory when maps change.
Instead, hard results use two explicit levels:

1. Robustness over all 100 frames: coverage, loss events, map switches and latency.
2. Conditional accuracy over one preselected common single-map interval,
   `12.9-14.9 s`, using the per-frame body trajectory for all modes. This interval was
   selected only from map continuity and coverage, before inspecting ATE/RPE.

The hard conditional result must always be reported with valid-frame counts. It is not
interchangeable with the full-sequence hard ATE.

## Environment

```bash
export A_ROOT=/path/to/ORB_SLAM3_A
export REPO_ROOT=/path/to/ORB_SLAM3
source "${REPO_ROOT}/scripts/A/activate_a_orb.sh"
```

Paths:

```text
A_ROOT=<workspace containing data, conda_envs, install, build>
REPO_ROOT=<this Git checkout>
runner=A_ROOT/build/A_baselines_repo/run_tartanair
config=REPO_ROOT/configs/A/tartanair.yaml
vocabulary=REPO_ROOT/Vocabulary/ORBvoc.txt
```

Create the environment and build:

```bash
conda env create -f "${REPO_ROOT}/configs/A/environment.yml"
export A_CONDA_ENV="$(conda run -n a_orb python -c 'import sys; print(sys.prefix)')"
bash "${REPO_ROOT}/scripts/A/build_pangolin.sh"
bash "${REPO_ROOT}/experiments/A_baselines/build_a.sh"
```

The build requires Pangolin v0.6 at `A_PANGOLIN_PREFIX`, defaulting to
`A_ROOT/install/pangolin`.

Download and extract the public TartanAir assets:

```bash
python "${REPO_ROOT}/experiments/A_baselines/download_tartanair_a.py" \
  --data-root "${A_ROOT}/data/TartanAirV2"
```

## Prepare a sequence

```bash
python "${REPO_ROOT}/experiments/A_baselines/prepare_sequence.py" \
  --sequence-root "${A_ROOT}/data/TartanAirV2/OldTownSummer/Data_easy/P000" \
  --start-frame 200 \
  --num-frames 100 \
  --output "${REPO_ROOT}/results/A/inputs_final/OldTownSummer/easy/P000_2p0s_9p9s"
```

The same command supports the P002 holdout and hard P001 case by changing
`--sequence-root`, `--start-frame`, and `--output`.

## Run one configuration

```bash
python "${REPO_ROOT}/experiments/A_baselines/run_baseline.py" \
  --binary "${A_ROOT}/build/A_baselines_repo/run_tartanair" \
  --vocab "${REPO_ROOT}/Vocabulary/ORBvoc.txt" \
  --settings "${REPO_ROOT}/configs/A/tartanair.yaml" \
  --mode rgb \
  --manifest "${REPO_ROOT}/results/A/inputs_final/OldTownSummer/easy/P000_2p0s_9p9s/manifest.csv" \
  --imu "${REPO_ROOT}/results/A/inputs_final/OldTownSummer/easy/P000_2p0s_9p9s/imu.csv" \
  --output "${REPO_ROOT}/results/A/runs/new_run" \
  --seed 0 \
  --orb-root "${REPO_ROOT}"
```

The runner is headless, uses `bUseViewer=false`, paces input at the timestamp rate,
and writes `trajectory_est.tum`, `trajectory_frames.tum`, `trajectory_orb_raw.tum`,
`tracking.csv`, `run.log`, and `run_meta.json`. `trajectory_est.tum` is the optimized
keyframe trajectory in TartanAir body/NED coordinates and is the only primary comparison
trajectory.

ATE and RPE status are independent. A sparse keyframe trajectory can therefore produce
a valid ATE while the requested 1 m RPE interval has no admissible pairs. This is
reported as `ate_status=ok` and `rpe_*_status=unavailable`; it does not invalidate ATE.

An empty or insufficient primary keyframe trajectory is reported as
`primary_trajectory_unavailable` with `keyframe_count` and `frame_pose_count`. The
per-frame trajectory and tracking log remain available for map-aware diagnostics.

## Evaluate

RGB uses Sim(3). RGB+IMU and RGB-D+IMU use SE(3). RPE uses a 1 m interval.

```bash
python "${REPO_ROOT}/experiments/A_baselines/evaluate_run.py" \
  --estimated RUN/trajectory_est.tum \
  --reference "${REPO_ROOT}/results/A/inputs_final/OldTownSummer/easy/P000_2p0s_9p9s/trajectory_gt.tum" \
  --tracking RUN/tracking.csv \
  --output RUN/metrics.json \
  --alignment sim3 \
  --plot RUN/trajectory_overlay.png
```

## Full reproduction

The full three-sequence matrix, hard common-window evaluation and result summaries are
reproduced with:

```bash
bash "${REPO_ROOT}/experiments/A_baselines/reproduce_baseline_v2.sh"
```

To run the remaining physical trajectories in the local package:

```bash
bash "${REPO_ROOT}/experiments/A_baselines/run_missing_sequences.sh"
python "${REPO_ROOT}/experiments/A_baselines/summarize_all_sequences.py" \
  --root "${REPO_ROOT}"
python "${REPO_ROOT}/experiments/A_baselines/plot_available_trajectories.py" \
  --root "${REPO_ROOT}"
python "${REPO_ROOT}/experiments/A_baselines/plot_case_bars.py" \
  --root "${REPO_ROOT}"
python "${REPO_ROOT}/experiments/A_baselines/plot_sensor_degradation_overview.py" \
  --root "${REPO_ROOT}"
```

To repair evaluation-only defects in an existing 54-run matrix without rerunning the
engine, use:

```bash
python "${REPO_ROOT}/experiments/A_baselines/reevaluate_existing_sequences.py" \
  "${REPO_ROOT}" \
  "${REPO_ROOT}/experiments/A_baselines/available_sequences.yaml" \
  "${A_CONDA_ENV}/bin/python"
```

`select_hard_window.py` is the audit utility used to find a common hard interval from
tracking validity and map continuity only. Its output is not used to choose a window by
the resulting ATE.

## Known results

The frozen handoff results are in:

```text
results/A/baseline_metrics.csv
results/A/baseline_metrics.json
results/A/baseline_aggregate.csv
results/A/baseline_aggregate.json
results/A/validation_metrics.csv
results/A/validation_metrics.json
results/A/validation_aggregate.csv
results/A/validation_aggregate.json
results/A/hard_metrics.csv
results/A/hard_metrics.json
results/A/hard_aggregate.csv
results/A/hard_aggregate.json
results/A/hard_common_metrics.csv
results/A/hard_common_metrics.json
results/A/hard_common_aggregate.csv
results/A/hard_common_aggregate.json
results/A/hard_map_segments.csv
results/A/hard_map_segments.json
results/A/hard_map_segment_summary.csv
results/A/hard_map_segment_summary.json
results/A/figures/baseline_summary.png
results/A/figures/validation_summary.png
results/A/figures/hard_robustness.png
results/A/figures/hard_common_summary.png
results/A/figures/hard_map_segments.png
results/A/figures/hard_rgb_overlay.png
results/A/figures/hard_rgb_imu_overlay.png
results/A/figures/hard_rgbd_imu_overlay.png
results/A/all_sequences_metrics.csv
results/A/all_sequences_aggregate.csv
results/A/available_sequence_trajectory_manifest.csv
results/A/figures/easy_available_trajectories.png
results/A/figures/hard_available_trajectories.png
```

Failed initialization and tracking runs remain in the CSV/JSON outputs. They are not
replaced by successful-only segments or by the largest map.
