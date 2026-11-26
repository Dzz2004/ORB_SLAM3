# A Presentation Package

This directory contains the presentation-ready materials for workstream A:

```text
presentation/A/
├─ README.md
├─ A_baseline_slides.md
├─ A_PR_BODY.md
├─ experiment_notes.md
├─ A_presentation_script.md
├─ data_slide_text.md
├─ sensor_degradation_slide.md
├─ docs/
│  └─ available_sequences_report.md
└─ figures/
   ├─ data/
   │  ├─ easy_three_cases_bars.png
   │  ├─ hard_three_cases_bars.png
   │  ├─ sensor_degradation_overview.png
   │  └─ per-case PNG/SVG charts
   ├─ baseline_summary.png
   ├─ validation_summary.png
   ├─ hard_robustness.png
   ├─ hard_common_summary.png
   ├─ hard_map_segments.png
   ├─ easy_rgb_overlay.png
   ├─ easy_rgb_imu_overlay.png
   ├─ easy_rgbd_imu_overlay.png
   ├─ validation_rgb_overlay.png
   ├─ validation_rgb_imu_overlay.png
   ├─ validation_rgbd_imu_overlay.png
   ├─ hard_rgb_overlay.png
   ├─ hard_rgb_imu_overlay.png
   ├─ hard_rgbd_imu_overlay.png
   ├─ easy_available_trajectories.png
   └─ hard_available_trajectories.png
```

## Suggested slide allocation

- Slide 3: ORB-SLAM3 baseline and research gap.
- Slide 5: shared TartanAir setup and evaluation protocol.
- Slide 6: Easy P000/P001/P002 bar-chart overview.
- Slide 7: Hard P000/P001/P002 bar-chart overview.
- Slide 8: metric interpretation, initialization failure and map discontinuity.
- Backup: per-case P000/P001/P002 charts in `presentation/A/figures/data/`.

## Required caveat

Do not state that fusion always improves accuracy. P000 and P002 produce different
orderings. Hard full-window ATE is unavailable because map frames are discontinuous;
the separate common-window result must always be shown with valid-frame coverage.
Also distinguish "no usable primary keyframe trajectory" from "zero tracking": every
one of the 54 runs has tracking poses, while only 32 have a continuous primary result.
