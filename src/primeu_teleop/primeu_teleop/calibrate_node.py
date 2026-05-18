#!/usr/bin/env python3
"""Calibration helper: capture the current tracker-to-robot offset.

When the user assumes a known reference pose (e.g. T-pose), this node
samples the live tracker poses and writes pos_offset / rot_offset values
into a file you can paste into the YAML config.

Usage:
    ros2 run primeu_teleop calibrate
    # then trigger a snapshot (Ctrl-C exits and writes the file)
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from scipy.spatial.transform import Rotation


class CalibrateNode(Node):
    """Sample tracker TFs and dump them as YAML offsets."""

    def __init__(self) -> None:
        super().__init__("primeu_teleop_calibrate")

        self.declare_parameter("base_frame", "libsurvive_world")
        self.declare_parameter("tracker_frames", [""])
        self.declare_parameter(
            "output_file", str(Path.home() / "primeu_calibration.yaml")
        )
        self.declare_parameter("sample_count", 50)

        self.base_frame: str = self.get_parameter("base_frame").value
        frames: list[str] = self.get_parameter("tracker_frames").value
        self.tracker_frames = [f for f in frames if f]
        self.output_file: str = self.get_parameter("output_file").value
        self.sample_count: int = self.get_parameter("sample_count").value

        if not self.tracker_frames:
            self.get_logger().error(
                "No tracker_frames given. Run with:\n"
                "  ros2 run primeu_teleop calibrate --ros-args "
                "-p tracker_frames:='[LHR-XXX, LHR-YYY]'"
            )
            sys.exit(1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.samples: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {
            f: [] for f in self.tracker_frames
        }

        self.timer = self.create_timer(0.05, self._sample)

        self.get_logger().info(
            f"Calibrating {len(self.tracker_frames)} trackers, "
            f"{self.sample_count} samples each. Hold the reference pose..."
        )

    def _sample(self) -> None:
        all_done = True
        for frame in self.tracker_frames:
            if len(self.samples[frame]) >= self.sample_count:
                continue
            all_done = False
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, frame, rclpy.time.Time()
                )
                pos = np.array(
                    [
                        tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z,
                    ]
                )
                rot = np.array(
                    [
                        tf.transform.rotation.w,
                        tf.transform.rotation.x,
                        tf.transform.rotation.y,
                        tf.transform.rotation.z,
                    ]
                )
                self.samples[frame].append((pos, rot))
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                pass

        if all_done:
            self._write_output()
            rclpy.shutdown()

    def _write_output(self) -> None:
        lines: list[str] = [
            "# Calibration snapshot — paste these offsets into your IK config",
            "# (primeu_libsurvive_mink_cfg.yaml) under the matching bone_name.",
            "",
        ]

        for frame, sample_list in self.samples.items():
            positions = np.stack([p for p, _ in sample_list])
            quats = np.stack([q for _, q in sample_list])
            mean_pos = positions.mean(axis=0)
            # Quaternion average (simple): use first then re-normalize
            ref_q = quats[0]
            mean_q = ref_q / np.linalg.norm(ref_q)
            euler = Rotation.from_quat(
                [mean_q[1], mean_q[2], mean_q[3], mean_q[0]]
            ).as_euler("xyz", degrees=True)

            lines.append(f"# Tracker: {frame}")
            lines.append(
                f"#   Mean position (m): "
                f"[{mean_pos[0]:.4f}, {mean_pos[1]:.4f}, {mean_pos[2]:.4f}]"
            )
            lines.append(
                f"#   Mean orientation (deg, xyz): "
                f"[{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}]"
            )
            lines.append(
                f"#   Quaternion (wxyz): "
                f"[{mean_q[0]:.4f}, {mean_q[1]:.4f}, "
                f"{mean_q[2]:.4f}, {mean_q[3]:.4f}]"
            )
            lines.append("")

        Path(self.output_file).write_text("\n".join(lines))
        self.get_logger().info(f"Calibration saved to {self.output_file}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
