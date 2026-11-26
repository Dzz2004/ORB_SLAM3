# A Baseline Presentation Notes

## Core message

We reproduced ORB-SLAM3's RGB, RGB+IMU and RGB-D+IMU modes on a controlled TartanAir
matrix. The same fixed windows, seeds and trajectory exporters are used for every mode.
Failed runs and map discontinuities remain visible.

## Controlled variables

- Same images, depth, IMU, GT, timestamps, seeds and ORB configuration.
- Same optimized-keyframe trajectory exporter for easy full-window accuracy.
- Coverage, loss events, map switches and latency use the complete tracking log.
- Full-window ATE/RPE is rejected when valid poses cross multiple maps.
- Hard numerical accuracy is a separate, preselected common single-map window.

## Easy P000

- RGB: 1.659 m ATE, 98% coverage.
- RGB+IMU: 0.090 m ATE on three valid seeds, 73.3% mean coverage; 1 m RPE is
  available on two seeds.
- RGB-D+IMU: 0.224 m ATE, 98% coverage.

## Easy P002 holdout

- RGB: 1.103 m ATE, 99% coverage.
- RGB+IMU: 0.326 m ATE, 99% coverage.
- RGB-D+IMU: 0.281 m ATE, 98% coverage.
- Both fusion modes beat RGB here, showing scene dependence.

## Hard P001 robustness

- Full-window ATE/RPE: N/A because valid poses span discontinuous map frames.
- Coverage: RGB 50.3%, RGB+IMU 28.0%, RGB-D+IMU 60.0%.
- Loss events: 4.7, 21.7 and 19.0 respectively.
- RGB-D+IMU has the shortest longest-loss segment.

Per-map segments are aligned independently in `hard_map_segments.csv`; they are never
concatenated into one trajectory.

- RGB: 4 evaluable segments across 3 seeds.
- RGB+IMU: no segment reaches the minimum continuous length.
- RGB-D+IMU: 3 evaluable short segments, with median ATE 0.211 m.

## Hard P001 conditional accuracy

The interval `12.9-14.9 s` was selected from map continuity and coverage only:

- RGB: 0.281 m ATE on 2/3 valid seeds; one seed has zero coverage.
- RGB+IMU: 1.415 m ATE on 3/3 seeds, 47.6% coverage.
- RGB-D+IMU: 1.262 m ATE on 3/3 seeds, 57.1% coverage.

Never show this table without its valid-frame coverage.

## Interpretation

Fusion does not universally outperform RGB. The easy holdout reverses the ordering,
while the hard sequence exposes initialization, tracking-loss and map-continuity
failures. These are valid baseline findings.
