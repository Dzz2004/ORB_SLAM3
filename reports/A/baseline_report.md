# A Baseline Report

## Scope

This report evaluates the three requested ORB-SLAM3 modes without modifying the optimizer
or fusion backend:

- `rgb`: `MONOCULAR`
- `rgb_imu`: `IMU_MONOCULAR`
- `rgbd_imu`: `IMU_RGBD`

The study uses TartanAir V2 `OldTownSummer` with one primary easy trajectory, one easy
holdout and one hard robustness trajectory. Every mode is run with seeds 0, 1 and 2.

## Experiment matrix

| Role | Sequence | Frames | Interval | Seeds |
|---|---|---:|---|---|
| Primary | `Data_easy/P000` | 200-299 | 20.0-29.9 s | 0, 1, 2 |
| Holdout | `Data_easy/P002` | 200-299 | 20.0-29.9 s | 0, 1, 2 |
| Hard robustness | `Data_hard/P001` | 100-199 | 10.0-19.9 s | 0, 1, 2 |

## Controlled protocol

- Camera: pinhole 640x640, `fx=fy=320`, `cx=cy=320`, zero distortion
- Image input: OpenCV BGR (`Camera.RGB: 0`)
- IMU: 100 Hz, fixed optical-to-NED `Tbc`, zero translation
- ORB features: 1250
- Math-library threads: OpenMP/OpenBLAS/MKL fixed to one
- Seed is applied again after ORB-SLAM3 construction to override the internal
  TwoViewReconstruction seed
- Primary trajectory: ORB-SLAM3 optimized keyframe trajectory converted to body/NED
- Full-window ATE/RPE: only when all valid frames in the fixed window belong to one map
- RGB alignment: Sim(3); inertial alignment: SE(3)
- RPE interval: 1 m

Coverage and loss events always use the complete tracking log, including failed frames.

## Easy primary result: P000

Values are mean and standard deviation across three seeds. RGB+IMU has only two usable
optimized trajectories, so its error values use two seeds while coverage and latency use
all three.

| Mode | Valid seeds | ATE [m] | RPE trans. [m] | RPE rot. [deg] | Coverage | Loss events | Mean latency [ms] | P95 latency [ms] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| rgb | 3/3 | 1.659 ± 0.021 | 1.270 ± 0.284 | 21.776 ± 5.539 | 98.0% | 0.0 | 29.33 ± 0.66 | 36.35 ± 0.38 |
| rgb_imu | 2/3 | 0.123 ± 0.004 | 0.608 ± 0.023 | 19.907 ± 3.616 | 73.3% ± 4.6% | 8.0 ± 1.7 | 39.40 ± 3.27 | 100.32 ± 17.07 |
| rgbd_imu | 3/3 | 0.224 ± 0.012 | 0.634 ± 0.003 | 25.027 ± 0.440 | 98.0% | 0.0 | 26.18 ± 0.10 | 32.71 ± 2.65 |

RGB+IMU has the lowest conditional ATE but loses substantially more frames. RGB-D+IMU
retains almost complete coverage while its ATE remains above RGB+IMU.

## Easy holdout: P002

P002 was not used to tune the configuration.

| Mode | Valid seeds | ATE [m] | RPE trans. [m] | RPE rot. [deg] | Coverage | Loss events | Mean latency [ms] | P95 latency [ms] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| rgb | 3/3 | 1.103 ± 0.032 | 1.767 ± 0.042 | 15.104 ± 0.843 | 99.0% | 0.0 | 31.26 ± 3.77 | 43.68 ± 3.28 |
| rgb_imu | 3/3 | 0.326 ± 0.002 | 0.766 ± 0.028 | 12.806 ± 0.282 | 99.0% | 0.0 | 26.19 ± 0.80 | 30.76 ± 0.94 |
| rgbd_imu | 3/3 | 0.281 ± 0.004 | 0.791 ± 0.023 | 10.325 ± 0.121 | 98.0% | 0.0 | 24.96 ± 0.70 | 30.15 ± 1.12 |

P002 reverses the easy/P000 ordering: both fusion modes outperform RGB. This is valid
scene-dependent behavior, not grounds for retuning on the holdout.

## Hard robustness: P001

A hard window may contain multiple ORB-SLAM3 maps. Each map has its own local world
frame, so concatenating poses from different maps would produce an invalid continuous
trajectory. The full 100-frame result is therefore reported as robustness:

| Mode | Coverage | Loss events | Map switches | Longest loss [frames] | Mean latency [ms] | P95 latency [ms] | Full-window ATE/RPE |
|---|---:|---:|---:|---:|---:|---:|---|
| rgb | 50.3% ± 16.1% | 4.7 ± 2.1 | 1.0 ± 0.0 | 38.0 ± 6.2 | 32.84 ± 3.86 | 63.74 ± 18.54 | N/A |
| rgb_imu | 28.0% ± 1.0% | 21.7 ± 0.6 | 0.0 | 8.7 ± 0.6 | 49.68 ± 1.85 | 95.87 ± 9.21 | N/A |
| rgbd_imu | 60.0% | 19.0 | 0.0 | 2.0 | 25.87 ± 0.24 | 34.89 ± 0.32 | N/A |

RGB-D+IMU has the largest full-window coverage and shortest longest-loss segment.
RGB+IMU has the lowest coverage and the highest loss-event count under this window.

Contiguous single-map segments are also evaluated separately in
`hard_map_segments.csv`. Each segment is aligned independently; segments are never
concatenated. The segment summary reports segment count and the median of evaluable
segment errors rather than pretending all maps share one frame.

| Mode | Segments | Evaluable segments | Seeds with evaluable segment | Median segment ATE [m] | Median segment RPE trans. [m] | Median segment RPE rot. [deg] |
|---|---:|---:|---:|---:|---:|---:|
| rgb | 17 | 4 | 3/3 | 0.445 | 0.496 | 2.460 |
| rgb_imu | 66 | 0 | 0/3 | N/A | N/A | N/A |
| rgbd_imu | 60 | 3 | 3/3 | 0.211 | 0.012 | 0.094 |

Most short RGB+IMU and RGB-D+IMU segments fail the minimum length because valid poses
alternate with losses. They are retained as `too_short`; only continuous segments with
at least eight tracked frames and six associated poses are evaluated.

## Hard conditional accuracy

To provide a numerical hard comparison without mixing maps, one common interval was
selected from map continuity and coverage only, before inspecting ATE/RPE:

```text
P001 absolute time: 12.9-14.9 s
21 camera timestamps
single map 0 required independently in every run
same per-frame body trajectory source for all modes
RGB Sim(3), inertial modes SE(3)
RPE interval: 5 frames
```

| Mode | Valid seeds | ATE [m] | RPE trans. [m] | RPE rot. [deg] | Coverage by seed | Lost frames |
|---|---:|---:|---:|---:|---|---|
| rgb | 2/3 | 0.281 ± 0.204 | 0.330 ± 0.366 | 2.377 ± 2.447 | 47.6%, 100%, 0% | 11, 0, 21 |
| rgb_imu | 3/3 | 1.415 ± 0.000 | 3.173 ± 0.000 | 14.452 ± 0.000 | 47.6%, 47.6%, 47.6% | 11, 11, 11 |
| rgbd_imu | 3/3 | 1.262 ± 0.006 | 2.045 ± 0.000 | 21.082 ± 0.000 | 57.1%, 57.1%, 57.1% | 9, 9, 9 |

This is a **conditional accuracy after successful tracking** result. It is not a
full-sequence ATE and must not be reported without the coverage column. RGB seed 2 is a
real failed window and remains in the output.

## Interpretation

- Sensor fusion is not guaranteed to outperform RGB on every sequence.
- Easy/P000 favors RGB+IMU in conditional ATE but with lower coverage.
- Easy/P002 favors both fusion modes over RGB in accuracy.
- Hard/P001 exposes initialization, tracking-loss and map-continuity failures.
- RGB-D+IMU is more available than RGB+IMU in the hard window, but still does not beat
  RGB conditional accuracy.
- No failed seed is replaced by a successful segment or by the largest map.

## Repeatability

A repeated easy/RGB+IMU run with the same seed still differed because ORB-SLAM3 uses
internal worker threads. The protocol is reproducible, but execution is not bitwise
deterministic. Seed variation must be interpreted together with scheduling variation.

## Limitations

- Only one `OldTownSummer` environment is evaluated.
- The primary and holdout windows are short fixed intervals rather than full sequences.
- Hard full-window ATE/RPE is undefined because maps do not share a world frame.
- The hard conditional interval contains a small number of matched frames, so its
  uncertainty is large.
- TartanAir IMU is synthetic and ORB noise parameters use an EuRoC-scale setting.
