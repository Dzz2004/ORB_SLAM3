# A Baseline Failure Cases

## Long-window map reset

On easy `P000`, a 60-second window produced many map resets. The estimated trajectory
was split across maps whose local coordinate frames were not connected. Concatenating
all returned poses produced ATE above 10 m even though early tracking was visually
correct.

The same issue appeared after roughly 10-30 seconds on other exploratory windows.

## Fast feed without timestamp pacing

The first runner fed images as quickly as possible. Local mapping could not keep up,
which increased tracking loss. The final runner waits according to the 10 Hz camera
timestamps.

## Inertial initialization

Inertial runs repeatedly logged:

```text
IMU is not or recently initialized
Reseting active map
```

The controlled easy/P000 window has 73.3% RGB+IMU coverage and 8.0 loss events. All
three seeds produce a connected optimized trajectory and ATE, but one trajectory is too
sparse for a valid 1 m RPE pair. Older reports incorrectly treated that RPE failure as
an ATE failure. RGB-D+IMU reaches 98% coverage and 0.224 m ATE. Monocular inertial scale
initialization remains the weakest easy-case component.

Easy/P001 RGB+IMU has the same evaluation-only issue: all three seeds have a primary
keyframe trajectory and ATE, while every 1 m RPE query is empty.

## Hard map continuity and initialization failures

Hard P001 produced repeated messages of the form:

```text
IMU is not or recently initialized. Reseting active map...
Fail to track local map!
```

RGB-D+IMU retained 60.0% coverage, RGB retained 50.3% and RGB+IMU retained 28.0%.
RGB executed a map switch in every seed; the inertial modes more often emitted invalid
poses without creating a usable optimized keyframe segment. A new ORB-SLAM3 map has its
own local world frame, so the full hard window is evaluated as robustness rather than a
continuous ATE trajectory.

## Same-seed non-determinism

A repeated easy/RGB+IMU run with the same seed still differed because ORB-SLAM3 uses
internal worker threads. Math-library thread pools are fixed, but the full SLAM execution
is not bitwise deterministic. Results must therefore use repeated seeds and report
variability rather than a single selected run.
