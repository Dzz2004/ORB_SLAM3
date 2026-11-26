# A Baseline Slide Material

## Slide: Controlled setup

- Three official ORB-SLAM3 modes: RGB, RGB+IMU and RGB-D+IMU.
- TartanAir V2 `OldTownSummer`.
- Easy P000 primary, easy P002 holdout, hard P001 robustness.
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

Use `results/A/figures/baseline_summary.png` and
`results/A/figures/validation_summary.png`.

- Easy P000: RGB+IMU has the lowest conditional ATE but lower coverage.
- Easy P002: both fusion modes outperform RGB.
- The ordering changes between P000 and P002, so no universal fusion advantage is
  claimed.

## Slide: Hard results

Use `results/A/figures/hard_robustness.png`.

- Full-window hard ATE/RPE is N/A because map frames are discontinuous.
- RGB coverage: 50.3%; RGB+IMU: 28.0%; RGB-D+IMU: 60.0%.
- RGB-D+IMU has the shortest longest-loss segment.
- RGB+IMU has the highest loss-event count and P95 latency.
- Per-map segmentation finds no evaluable continuous RGB+IMU segment.

Use `results/A/figures/hard_common_summary.png` for conditional accuracy.

- The 12.9-14.9 s interval is the same for all modes and remains on map 0.
- RGB seed 2 has zero coverage in this interval and is retained as a failure.
- Conditional accuracy is never shown without its valid-frame coverage.

## Slide: Conclusions

- Official ORB-SLAM3 is a multi-map system; map reset is not one continuous world frame.
- Easy results are scene-dependent.
- Hard results show substantial loss and initialization failure.
- The data do not support a claim that fusion always improves accuracy.
- The next branches can reuse the same manifest, runner, tracking log and map-aware
  evaluation protocol.
