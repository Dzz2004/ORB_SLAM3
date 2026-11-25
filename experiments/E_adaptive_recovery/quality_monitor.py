"""Score available RGB and depth evidence on a common 0–1 scale."""


class QualityMonitor:
    def __init__(self, config):
        self.config = config
        self.previous = None

    def update(self, timestamp, slam_ok, rgb, depth, inliers):
        if rgb is None:
            rgb = (min(1.0, inliers / self.config["inlier_reference"])
                   if inliers is not None else float(slam_ok))
        if inliers is not None:
            rgb = min(rgb, inliers / self.config["inlier_reference"])
        if not slam_ok:
            rgb = 0.0
        # A downward change takes effect immediately. Upward changes are smoothed.
        if self.previous is not None:
            last_time, last_rgb, last_depth = self.previous
            if timestamp - last_time <= self.config["max_frame_gap_s"]:
                alpha = self.config["quality_alpha"]
                rgb = min(rgb, alpha * rgb + (1 - alpha) * last_rgb)
                if depth is not None and last_depth is not None:
                    depth = min(depth, alpha * depth + (1 - alpha) * last_depth)
        self.previous = (timestamp, rgb, depth)
        return rgb, depth
