#!/usr/bin/env python3
"""TF Inspector: 列出当前所有可用的 TF frames，帮助识别 tracker。

Usage:
    ros2 run primeu_teleop tf_inspector
    ros2 run primeu_teleop tf_inspector --ros-args -p duration:=10.0
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TFInspector(Node):
    """监听 /tf 和 /tf_static 话题，收集所有 frame 名称。"""

    def __init__(self) -> None:
        super().__init__("tf_inspector")

        self.declare_parameter("duration", 5.0)
        self.duration = self.get_parameter("duration").value

        self.dynamic_frames: dict[str, str] = {}
        self.static_frames: dict[str, str] = {}

        self.create_subscription(TFMessage, "/tf", self._tf_cb, 10)
        self.create_subscription(TFMessage, "/tf_static", self._tf_static_cb, 10)

        self.get_logger().info(
            f"Listening for TF frames for {self.duration:.1f}s..."
        )

        self.create_timer(self.duration, self._report_and_exit)

    def _tf_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.dynamic_frames[t.child_frame_id] = t.header.frame_id

    def _tf_static_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.static_frames[t.child_frame_id] = t.header.frame_id

    def _report_and_exit(self) -> None:
        print("\n" + "=" * 70)
        print("  TF Frame Inspection Report")
        print("=" * 70)

        if self.static_frames:
            print("\n[Static frames] (likely lighthouses / base stations)")
            for child, parent in sorted(self.static_frames.items()):
                print(f"  {parent:30s} -> {child}")

        if self.dynamic_frames:
            print("\n[Dynamic frames] (likely trackers)")
            for child, parent in sorted(self.dynamic_frames.items()):
                print(f"  {parent:30s} -> {child}")

        if not self.dynamic_frames and not self.static_frames:
            print("\n  No TF frames received.")
            print("  Is libsurvive_ros2 running?")
            print("    ros2 launch libsurvive_ros2 libsurvive_ros2.launch.py")

        print("\n" + "=" * 70)
        print("  Copy the tracker IDs (e.g. LHR-XXXXXXXX) into your config:")
        print("    primeu_teleop/config/primeu_libsurvive_mink_cfg.yaml")
        print("=" * 70 + "\n")

        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TFInspector()
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
