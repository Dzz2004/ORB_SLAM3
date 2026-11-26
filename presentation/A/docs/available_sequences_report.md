# A Baseline: All Available Trajectory Sequences

## Scope

The local `OldTownSummer` TartanAir V2 package contains three Easy and three Hard trajectory directories. This matrix uses all six directories with RGB, RGB+IMU and RGB-D+IMU for seeds 0, 1 and 2.

This produces `6 trajectories x 3 modes x 3 seeds = 54` runs.

## Evaluation rules

- Use the optimized-keyframe body trajectory as the primary comparison source.
- RGB uses Sim(3); inertial modes use SE(3).
- Full-window ATE/RPE require a single map and a non-empty primary trajectory.
- ATE and RPE are evaluated independently; RPE unavailability does not erase ATE.
- Coverage, loss events and latency always use all 100 tracking frames.
- Per-frame by-map plots are diagnostics, never substitutes for the primary continuous trajectory.

## Status accounting

- Successful continuous primary evaluation: `32` runs.
- No usable primary keyframe trajectory: `18` runs.
- Primary trajectory exists but the window crosses maps: `4` runs.
- Tracking poses were available in `54` of `54` runs, including failed primary-trajectory runs.

`primary_trajectory_unavailable` is not the same as zero tracking frames. It means ORB-SLAM3 produced no usable optimized keyframe trajectory (for example zero keyframes), while per-frame diagnostic poses may still exist.

## Easy results

Means use only seeds with a valid ATE or RPE. Coverage and loss events use all three seeds.

| Trajectory | Mode | Primary | ATE seeds | RPE-T seeds | RPE-R seeds | ATE [m] | RPE-T [m] | RPE-R [deg] | Coverage | Loss events |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| P000 | rgb | 3/3 | 3/3 | 3/3 | 3/3 | 1.659 ± 0.021 | 1.270 ± 0.284 | 21.776 ± 5.539 | 98.0% | 0.0 |
| P000 | rgb_imu | 3/3 | 3/3 | 2/3 | 2/3 | 0.090 ± 0.057 | 0.608 ± 0.023 | 19.907 ± 3.616 | 73.3% | 8.0 |
| P000 | rgbd_imu | 3/3 | 3/3 | 3/3 | 3/3 | 0.224 ± 0.012 | 0.634 ± 0.003 | 25.027 ± 0.440 | 98.0% | 0.0 |
| P001 | rgb | 3/3 | 3/3 | 3/3 | 3/3 | 2.635 ± 0.088 | 2.013 ± 0.068 | 9.476 ± 0.112 | 95.0% | 0.0 |
| P001 | rgb_imu | 3/3 | 3/3 | 0/3 | 0/3 | 0.300 ± 0.254 | N/A | N/A | 58.7% | 14.3 |
| P001 | rgbd_imu | 3/3 | 3/3 | 3/3 | 3/3 | 0.235 ± 0.022 | 1.296 ± 0.005 | 11.603 ± 0.969 | 98.0% | 0.0 |
| P002 | rgb | 3/3 | 3/3 | 3/3 | 3/3 | 1.103 ± 0.032 | 1.767 ± 0.042 | 15.104 ± 0.843 | 99.0% | 0.0 |
| P002 | rgb_imu | 3/3 | 3/3 | 3/3 | 3/3 | 0.326 ± 0.002 | 0.766 ± 0.028 | 12.806 ± 0.282 | 99.0% | 0.0 |
| P002 | rgbd_imu | 3/3 | 3/3 | 3/3 | 3/3 | 0.281 ± 0.004 | 0.791 ± 0.023 | 10.325 ± 0.121 | 98.0% | 0.0 |

## Hard results

Means use only seeds with a valid ATE or RPE. Coverage and loss events use all three seeds.

| Trajectory | Mode | Primary | ATE seeds | RPE-T seeds | RPE-R seeds | ATE [m] | RPE-T [m] | RPE-R [deg] | Coverage | Loss events |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| P000 | rgb | 3/3 | 3/3 | 3/3 | 3/3 | 0.588 ± 0.024 | 1.243 ± 0.044 | 18.563 ± 1.078 | 91.0% | 0.0 |
| P000 | rgb_imu | 1/3 | 1/3 | 0/3 | 0/3 | 0.102 ± 0.000 | N/A | N/A | 24.7% | 19.7 |
| P000 | rgbd_imu | 0/3 | 0/3 | 0/3 | 0/3 | N/A | N/A | N/A | 58.0% | 20.0 |
| P001 | rgb | 2/3 | 0/3 | 0/3 | 0/3 | N/A | N/A | N/A | 50.3% | 4.7 |
| P001 | rgb_imu | 0/3 | 0/3 | 0/3 | 0/3 | N/A | N/A | N/A | 28.0% | 21.7 |
| P001 | rgbd_imu | 0/3 | 0/3 | 0/3 | 0/3 | N/A | N/A | N/A | 60.0% | 19.0 |
| P002 | rgb | 3/3 | 1/3 | 1/3 | 1/3 | 4.877 ± 0.000 | 2.306 ± 0.000 | 17.660 ± 0.000 | 76.3% | 1.3 |
| P002 | rgb_imu | 0/3 | 0/3 | 0/3 | 0/3 | N/A | N/A | N/A | 35.0% | 21.3 |
| P002 | rgbd_imu | 0/3 | 0/3 | 0/3 | 0/3 | N/A | N/A | N/A | 68.7% | 14.7 |

## Output files

```text
results/A/all_sequences_metrics.csv
results/A/all_sequences_metrics.json
results/A/all_sequences_aggregate.csv
results/A/all_sequences_aggregate.json
results/A/available_sequence_trajectory_manifest.csv
results/A/available_sequences/trajectories/
results/A/figures/easy_available_trajectories.png
results/A/figures/hard_available_trajectories.png
```

## Limitations

- The package contains three physical trajectories per difficulty, not five.
- Windows are fixed 100-frame samples, not complete sequence lengths.
- Hard runs frequently reset maps or fail inertial initialization.
- Easy P001 RGB+IMU ATE is available in all three runs, but 1 m RPE pairs are unavailable because the exported keyframe trajectory is too sparse.
