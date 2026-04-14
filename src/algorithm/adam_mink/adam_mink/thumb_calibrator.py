"""Time-windowed thumb joint calibrator.

Records per-target (left/right x thumb_1/thumb_2) min/max joint-angle sums
over a configurable duration, then applies the results as new mapping limits.

Usage from a ROS node::

    calibrator = ThumbCalibrator(logger=self.get_logger())

    # When the user triggers calibration:
    calibrator.start(duration_sec=5.0)

    # Every control-loop tick, feed current joint-angle sums:
    if calibrator.collect(joint_sums):
        # calibration just finished - rebuild your finger params
        ...
"""

import time
from typing import Protocol

DEFAULT_CALIBRATE_DURATION_SEC = 5.0
MIN_RANGE_THRESHOLD = 1e-3


class LoggerLike(Protocol):
    """Minimal logger interface (compatible with both rclpy and stdlib)."""

    def info(self, msg: str) -> None: ...
    def warning(self, msg: str) -> None: ...


# target_name -> joint names whose angles are summed for that target
CALIBRATION_TARGETS: dict[str, list[str]] = {
    "hand_thumb_1_Left": [
        "L_thumb_MCP_joint2",
        "L_thumb_PIP_joint",
        "L_thumb_DIP_joint",
    ],
    "hand_thumb_2_Left": ["L_thumb_MCP_joint1"],
    "hand_thumb_1_Right": [
        "R_thumb_MCP_joint2",
        "R_thumb_PIP_joint",
        "R_thumb_DIP_joint",
    ],
    "hand_thumb_2_Right": ["R_thumb_MCP_joint1"],
}


class ThumbCalibrator:
    """Collects joint-angle samples over a time window and derives min/max limits.

    The calibrator is deliberately ROS-agnostic.  The host node is responsible
    for declaring/resetting ROS parameters and rebuilding its own finger
    handle params when :meth:`collect` returns ``True``.
    """

    def __init__(
        self,
        default_limits: dict[str, list[float]],
        logger: LoggerLike | None = None,
    ) -> None:
        self.limits: dict[str, list[float]] = {k: list(v) for k, v in default_limits.items()}
        self._logger = logger
        self._active = False
        self._end_time = 0.0
        self._stats: dict[str, dict[str, float]] = {}

    @property
    def active(self) -> bool:
        return self._active

    def start(self, duration_sec: float) -> None:
        """Begin a new calibration window of *duration_sec* seconds."""
        self._active = True
        self._end_time = time.monotonic() + duration_sec
        self._stats = {
            name: {"min": float("inf"), "max": float("-inf")} for name in CALIBRATION_TARGETS
        }
        if self._logger:
            self._logger.info(f"Thumb calibration started, collecting for {duration_sec:.2f}s")

    def collect(self, joint_sums: dict[str, float]) -> bool:
        """Feed one sample per target.

        Args:
            joint_sums: Mapping of *target_name* → current summed joint angle.

        Returns:
            ``True`` exactly once, on the tick when calibration finishes.
        """
        if not self._active:
            return False

        for target_name, value in joint_sums.items():
            if target_name in self._stats:
                self._stats[target_name]["min"] = min(self._stats[target_name]["min"], value)
                self._stats[target_name]["max"] = max(self._stats[target_name]["max"], value)

        if time.monotonic() >= self._end_time:
            self._finalize()
            return True
        return False

    def _finalize(self) -> None:
        self._active = False
        for target_name, stats in self._stats.items():
            min_val = stats["min"]
            max_val = stats["max"]
            if (max_val - min_val) > MIN_RANGE_THRESHOLD:
                self.limits[target_name] = [min_val, max_val]
                if self._logger:
                    self._logger.info(
                        f"Calibrated {target_name}: min={min_val:.4f}, max={max_val:.4f}"
                    )
            else:
                if self._logger:
                    self._logger.warning(
                        f"Skipped {target_name} calibration: insufficient range "
                        f"(min={min_val:.4f}, max={max_val:.4f})"
                    )
