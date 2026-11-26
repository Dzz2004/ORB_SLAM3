#!/usr/bin/env python3
"""Select a common hard interval using tracking validity and map continuity only."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


MODES = ("rgb", "rgb_imu", "rgbd_imu")


def read_tracking(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def candidate_windows(
    rows: list[dict[str, str]], window_frames: int
) -> list[dict[str, object]]:
    candidates = []
    for start_index in range(0, len(rows) - window_frames + 1):
        window = rows[start_index : start_index + window_frames]
        valid = [row for row in window if row["valid"] == "1"]
        maps = sorted({int(row["map_id"]) for row in valid if int(row["map_id"]) >= 0})
        if maps != [0]:
            continue
        coverage = len(valid) / window_frames
        candidates.append(
            {
                "start_index": start_index,
                "start": float(window[0]["timestamp"]),
                "end": float(window[-1]["timestamp"]),
                "valid_frames": len(valid),
                "coverage": coverage,
                "lost_frames": window_frames - len(valid),
            }
        )
    return candidates


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan-root", required=True, type=Path)
    parser.add_argument("--window-frames", default=21, type=int)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    scan_root = args.scan_root.resolve()

    results = []
    for run_root in sorted((scan_root / "runs").iterdir()):
        if not run_root.is_dir():
            continue
        mode_windows = {}
        for mode in MODES:
            tracking = run_root / mode / "tracking.csv"
            mode_windows[mode] = {
                window["start_index"]: window
                for window in candidate_windows(
                    read_tracking(tracking), args.window_frames
                )
            }
        common_indexes = set.intersection(
            *(set(windows) for windows in mode_windows.values())
        )
        if not common_indexes:
            continue

        def score(index: int) -> tuple[float, float, float]:
            selected = [mode_windows[mode][index] for mode in MODES]
            return (
                min(float(window["coverage"]) for window in selected),
                sum(float(window["coverage"]) for window in selected) / len(selected),
                -sum(int(window["lost_frames"]) for window in selected),
            )

        selected_index = max(common_indexes, key=score)
        selected = {
            mode: mode_windows[mode][selected_index] for mode in MODES
        }
        results.append(
            {
                "run": run_root.name,
                "selection_rule": (
                    "maximize minimum per-mode coverage; require one common map 0 "
                    f"over {args.window_frames} frames"
                ),
                "modes": selected,
            }
        )

    if not results:
        raise SystemExit("No common single-map window found")
    best = max(
        results,
        key=lambda result: (
            min(float(result["modes"][mode]["coverage"]) for mode in MODES),
            sum(float(result["modes"][mode]["coverage"]) for mode in MODES) / 3,
        ),
    )
    payload = {"best": best, "all_runs": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(best, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
