# A Baseline Slide Material

## Slide: Controlled setup

- Three official ORB-SLAM3 modes: RGB, RGB+IMU and RGB-D+IMU.
- TartanAir V2 `OldTownSummer`.
- Easy and Hard P000/P001/P002, all six physical trajectories in the local package.
- Three seeds per sequence and mode.
- Same camera, ORB configuration, IMU parameters, timestamps and trajectory exporter.
- Headless execution with single-thread math libraries.

## Slide: Fair evaluation

- RGB uses Sim(3); inertial modes use SE(3).
- Full-window ATE/RPE requires all valid frames to remain in one map.
- Coverage, loss events, map switches and latency always use every frame.
- Failed seeds and failed map-continuity checks remain in the result files.
- Hard numerical accuracy uses one preselected common single-map interval.

## Slide: Easy results

Use `presentation/A/figures/data/easy_three_cases_bars.png`.

- Easy P000: RGB+IMU has the lowest conditional ATE but lower coverage.
- Easy P001: RGB+IMU has ATE in all three seeds, but 1 m RPE pairs are unavailable.
- Easy P002: both fusion modes outperform RGB.
- The ordering changes between P000 and P002, so no universal fusion advantage is
  claimed.
- Never replace `N/A` with zero; show the valid seed count above every bar.

## Slide: Hard results

Use `presentation/A/figures/data/hard_three_cases_bars.png`.

- Full-window hard ATE/RPE is N/A because map frames are discontinuous.
- RGB coverage: 50.3%; RGB+IMU: 28.0%; RGB-D+IMU: 60.0%.
- RGB-D+IMU has the shortest longest-loss segment.
- RGB+IMU has the highest loss-event count and P95 latency.
- Per-map segmentation finds no evaluable continuous RGB+IMU segment.
- Across all 54 runs, 32 have a continuous primary result, 18 have no usable primary
  keyframe trajectory, and 4 cross maps; all 54 still contain per-frame tracking data.

Use `results/A/figures/hard_common_summary.png` for conditional accuracy.

- The 12.9-14.9 s interval is the same for all modes and remains on map 0.
- RGB seed 2 has zero coverage in this interval and is retained as a failure.
- Conditional accuracy is never shown without its valid-frame coverage.

## Slide: Metric interpretation

- ATE and RPE are reported independently; RPE unavailability does not invalidate ATE.
- The current primary trajectory is optimized keyframes, not the official largest-map
  full-frame trajectory.
- RGB and fusion modes therefore use different numbers of poses.
- Treat the current bars as a baseline diagnostic; recompute official full-frame
  trajectories before claiming a final cross-mode accuracy ranking.

## Slide: Conclusions

- Official ORB-SLAM3 is a multi-map system; map reset is not one continuous world frame.
- Easy results are scene-dependent.
- Hard results show substantial loss and initialization failure.
- The data do not support a claim that fusion always improves accuracy.
- The next branches can reuse the same manifest, runner, tracking log and map-aware
  evaluation protocol.
