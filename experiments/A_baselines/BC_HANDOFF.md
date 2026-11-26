# B/C Interface Handoff

Use the same ORB commit, Conda environment, YAML, camera intrinsics, and trajectory
window. Do not build a separate ORB fork for comparison.

## Required modes

- B: `rgb` and `rgb_imu`
- C: `rgb_imu` and `rgbd_imu`

## Required inputs

Every run uses the same manifest schema:

```text
timestamp,rgb_path,depth_path,gt_index,frame_id
```

Every run uses the same IMU schema:

```text
timestamp,gx,gy,gz,ax,ay,az
```

B may replace `rgb_path` with blurred, illuminated, or occluded images. C may replace
`depth_path` with corrupted float32-encoded depth images. Frame count, frame IDs,
timestamps, and ordering must remain unchanged.

## Depth contract

C should keep the TartanAir lossless encoding:

- `640x640x4` PNG
- `float32` decoded with `png.view("<f4")`
- units in meters
- invalid depth may be encoded as zero

The runner also accepts a one-channel `CV_32FC1` depth in memory, but the current file
adapter reads the lossless four-channel PNG format.

## Reproducible call

Use `prepare_sequence.py` with `--rgb-subdir` or `--depth-subdir` pointing to the
corrupted data directory. Then pass the resulting manifest and the untouched IMU CSV
to `run_baseline.py`.

## Evaluation contract

- `rgb`: Sim(3)
- `rgb_imu`: SE(3)
- `rgbd_imu`: SE(3)
- Report ATE, RPE, valid-pose coverage, lost events, mean latency, and P95 latency.
- Report orientation APE because RPE rotation is invariant to some global frame offsets.
- Use `trajectory_est.tum`, which is in the TartanAir body/NED frame.
- `trajectory_est.tum` is always the same primary optimized-keyframe body trajectory.
  Do not switch trajectory exporters for only one arm of a comparison.
- `trajectory_frames.tum` is a diagnostic per-frame trajectory and must not replace the
  primary keyframe trajectory in a comparison.
- Only compare ATE/RPE when `metric_status` is `ok`. If it is
  `invalid_map_discontinuity`, coverage and lost events remain available but ATE
  must be reported as `NA`.
- Failed initialization must be preserved as a failed run.
- Hard full-window results may contain multiple maps. Report coverage, loss events,
  map switches and latency for the full window; do not concatenate map coordinates.
- The frozen hard conditional interval is `12.9-14.9 s` within the P001 100-frame
  window. It uses the per-frame body trajectory for all modes and RPE over 5 frames.

## Minimum handoff smoke test

For the frozen A handoff:

1. Run clean `rgb`.
2. Run one B-corrupted `rgb`.
3. Run clean `rgb_imu`.
4. Run one C-corrupted `rgbd_imu`.

Run this smoke test first on easy P000 and then on the hard P001 window. The hard test
must retain failed runs instead of forcing a successful single-map segment.

All four must produce a `metrics.json`. A low metric may be a valid result; silently
deleting failed frames or runs is not allowed.
