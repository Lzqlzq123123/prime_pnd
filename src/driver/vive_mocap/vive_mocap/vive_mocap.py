import json
import socket
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from vive_mocap.utils import (
    _mat_mul,
    _mat_vec_mul,
    _matrix_to_quat,
    _quat_to_matrix,
    invert,
    matrix_to_pose,
    pose_to_matrix,
)


@dataclass
class PositionOffset:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class RotationOffset:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class TransformOffset:
    position: PositionOffset = field(default_factory=PositionOffset)
    rotation: RotationOffset = field(default_factory=RotationOffset)


EXPECTED_BONES: list[str] = [
    "Waist",
    "HMD",
    "LeftElbow",
    "LeftController",
    "RightElbow",
    "RightController",
]


class ViveMocap(Node):
    def __init__(self) -> None:
        super().__init__("vive_mocap")

        self._frames: dict[str, TransformStamped] = {}
        for name in EXPECTED_BONES:
            stamped = TransformStamped()
            stamped.header.frame_id = "world"
            stamped.child_frame_id = name
            stamped.transform.translation.x = 0.0
            stamped.transform.translation.y = 0.0
            stamped.transform.translation.z = 0.0
            stamped.transform.rotation.x = 0.0
            stamped.transform.rotation.y = 0.0
            stamped.transform.rotation.z = 0.0
            stamped.transform.rotation.w = 1.0
            self._frames[name] = stamped

        self._tf_broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.calibrated_pub = self.create_publisher(Bool, "vive_mocap/calibrated", qos)
        self.transform_offsets = {name: TransformOffset() for name in EXPECTED_BONES}
        self.zero_offset = {
            "Waist": 1.0 + 0.25,
            "HMD": 1.0
            + 0.7
            - 0.2,  # when the vr is worn around the neck, the head is 0.2m away from the body
            "LeftElbow": 1.0 + 0.4,
            "LeftController": 1.0 + -0.05,
            "RightElbow": 1.0 + 0.4,
            "RightController": 1.0 + -0.05,
        }

        self._unity_to_robot = [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]
        self._unity_to_robot_T = [
            [
                self._unity_to_robot[0][0],
                self._unity_to_robot[1][0],
                self._unity_to_robot[2][0],
            ],
            [
                self._unity_to_robot[0][1],
                self._unity_to_robot[1][1],
                self._unity_to_robot[2][1],
            ],
            [
                self._unity_to_robot[0][2],
                self._unity_to_robot[1][2],
                self._unity_to_robot[2][2],
            ],
        ]

        self.calibrated = False
        self.robot_scale = 1.0
        self.robot_arm_length = 0.58
        self.T_tracker_chest = np.eye(4)
        self.T_tracker_upperL = np.eye(4)
        self.T_tracker_upperR = np.eye(4)

        self._use_json_input = self.declare_parameter("use_json_input", True).value
        self._json_input_file = self.declare_parameter(
            "json_input_file", "tests/mocap_data/vive_mocap_data.json"
        ).value
        self._json_rate_hz = float(self.declare_parameter("json_rate_hz", 120.0 * 0.7).value)

        if not self._use_json_input:
            self.udp_ip = "0.0.0.0"
            # Use a different default port from vr_mocap to avoid conflict
            self.udp_port = 12070
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.udp_ip, self.udp_port))
            self.get_logger().info(
                f"Vive mocap UDP receiver listening on {self.udp_ip}:{self.udp_port}"
            )
        else:
            self.sock = None
            self.get_logger().info(
                f"Vive mocap reading from JSON file {self._json_input_file} at {self._json_rate_hz:.1f} Hz"
            )

        self._save_json = self.declare_parameter("save_json", False).value
        self._json_file = self.declare_parameter("json_file", "vive_mocap_data.json").value
        if self._save_json:
            self.get_logger().info(f"Saving raw Vive data to {self._json_file}")

        if self._use_json_input:
            self._recv_thread = threading.Thread(target=self.file_loop, daemon=True)
        else:
            self._recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self._recv_thread.start()

    def _get_frame_pose(self, name: str) -> tuple[list[float], list[float]]:
        frame = self._frames[name].transform
        t_in = [frame.translation.x, frame.translation.y, frame.translation.z]
        q_in = [frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w]
        return t_in, q_in

    def _set_frame_pose(self, name: str, t: list[float], q: list[float]) -> None:
        frame = self._frames[name].transform
        frame.translation.x = float(t[0])
        frame.translation.y = float(t[1])
        frame.translation.z = float(t[2])
        frame.rotation.x = float(q[0])
        frame.rotation.y = float(q[1])
        frame.rotation.z = float(q[2])
        frame.rotation.w = float(q[3])

    def _apply_calibration_transform(self, name: str) -> None:
        tracker_by_frame = {
            "Waist": self.T_tracker_chest,
            "RightElbow": self.T_tracker_upperR,
            "LeftElbow": self.T_tracker_upperL,
        }
        tracker_transform = tracker_by_frame.get(name)
        if tracker_transform is None:
            return

        t_in, q_in = self._get_frame_pose(name)
        transform_in = pose_to_matrix(t_in, q_in)
        transform_out = transform_in @ tracker_transform
        t_out, q_out = matrix_to_pose(transform_out)
        self._set_frame_pose(name, list(t_out), q_out)

    def _apply_root_relative_scaling(self, names: list[str], root_name: str = "Waist") -> None:
        """Scale joint positions around a fixed root, preserving pose structure."""
        root_frame = self._frames.get(root_name)
        if root_frame is None:
            return

        root = np.array(
            [
                root_frame.transform.translation.x,
                root_frame.transform.translation.y,
                root_frame.transform.translation.z,
            ],
            dtype=float,
        )

        for name in names:
            frame = self._frames.get(name)
            if frame is None:
                continue

            p = np.array(
                [
                    frame.transform.translation.x,
                    frame.transform.translation.y,
                    frame.transform.translation.z,
                ],
                dtype=float,
            )
            p_scaled = root + self.robot_scale * (p - root)
            frame.transform.translation.x = float(p_scaled[0])
            frame.transform.translation.y = float(p_scaled[1])
            frame.transform.translation.z = float(p_scaled[2])

    def _update_frame(
        self,
        name: str,
        position: dict[str, float],
        orientation: dict[str, float],
    ) -> None:
        if name not in self._frames:
            stamped = TransformStamped()
            stamped.header.frame_id = "world"
            stamped.child_frame_id = name
            self._frames[name] = stamped

        frame = self._frames[name]

        # Raw Unity-style coordinates from Vive/SteamVR:
        # x: right, y: up, z: forward (+Z)
        x, y, z = (
            float(position["x"]),
            float(position["y"]),
            float(position["z"]),
        )
        qx, qy, qz, qw = (
            float(orientation["x"]),
            float(orientation["y"]),
            float(orientation["z"]),
            float(orientation["w"]),
        )

        # Position: map Unity VR -> robot frame using a fixed 3x3 transform.
        rx, ry, rz = _mat_vec_mul(self._unity_to_robot, [x, y, z])
        frame.transform.translation.x = rx
        frame.transform.translation.y = ry
        frame.transform.translation.z = rz

        # Orientation: convert quaternion -> matrix, change basis with the same
        # Unity -> robot transform, then convert back to quaternion.
        ru = _quat_to_matrix(qx, qy, qz, qw)

        # R_robot = P * R_unity * P^T
        p = self._unity_to_robot
        rr = _mat_mul(_mat_mul(p, ru), self._unity_to_robot_T)
        rqx, rqy, rqz, rqw = _matrix_to_quat(rr)

        frame.transform.rotation.x = rqx
        frame.transform.rotation.y = rqy
        frame.transform.rotation.z = rqz
        frame.transform.rotation.w = rqw

        # Apply calibration transforms
        self._apply_calibration_transform(name)

        offset = self.transform_offsets[name].position
        frame.transform.translation.x -= offset.x
        frame.transform.translation.y -= offset.y
        frame.transform.translation.z += offset.z

    def receive_loop(self) -> None:
        while rclpy.ok():
            data, addr = self.sock.recvfrom(1024 * 2)
            time_stamp = self.get_clock().now()
            try:
                payload = json.loads(data.decode())
            except Exception as exc:
                self.get_logger().warn(f"Failed to decode JSON from {addr}: {exc}")
                continue

            self._maybe_save_payload(payload, time_stamp.nanoseconds)
            self._process_payload(payload, time_stamp)

    def _maybe_save_payload(self, payload: dict, stamp_ns: int) -> None:
        if not self._save_json:
            return
        try:
            record = {"stamp": int(stamp_ns), "payload": payload}
            with open(self._json_file, "a", encoding="utf-8") as f:
                json.dump(record, f, ensure_ascii=False)
                f.write("\n")
        except Exception as exc:
            self.get_logger().warn(f"Failed to write Vive JSON to {self._json_file}: {exc}")

    def _process_payload(self, payload: dict, time_stamp) -> None:
        """Common processing for a single Vive payload dict."""
        trackers = payload.get("trackers", [])
        updated_names: list[str] = []
        for tracker in trackers:
            is_tracked = tracker.get("isTracked", False)
            if not is_tracked and tracker["name"] != "HMD":
                self.get_logger().warn(f"Tracker {tracker['name']} is not tracked")
                # TODO: tracker missing, need reset calibration
            self._update_frame(tracker["name"], tracker["pos"], tracker["rot"])
            updated_names.append(tracker["name"])
            self._frames[tracker["name"]].header.stamp = time_stamp.to_msg()

        self._handle_calibration(payload)
        self._apply_root_relative_scaling(updated_names)
        self._tf_broadcaster.sendTransform(list(self._frames.values()))

    def file_loop(self) -> None:
        """Replay Vive data from a JSONL file at a fixed rate."""
        if not self._json_input_file:
            self.get_logger().fatal("json_input_file is empty; cannot replay Vive data.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Starting Vive JSON replay from {self._json_input_file}")
        period = 1.0 / self._json_rate_hz if self._json_rate_hz > 0 else 0.0

        while rclpy.ok():
            try:
                with open(self._json_input_file, encoding="utf-8") as f:
                    for line in f:
                        if not rclpy.ok():
                            break
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            record = json.loads(line)
                        except Exception as exc:
                            self.get_logger().warn(f"Failed to parse JSON line: {exc}")
                            continue

                        payload = record.get("payload", record)
                        time_stamp = self.get_clock().now()
                        self._process_payload(payload, time_stamp)

                        if period > 0:
                            time.sleep(period)
            except FileNotFoundError:
                self.get_logger().error(f"JSON input file not found: {self._json_input_file}")
                break
            except Exception as exc:
                self.get_logger().error(
                    f"Error while reading JSON input file {self._json_input_file}: {exc}"
                )
                break

    def reset_calibration(self) -> None:
        self.robot_scale = 1.0
        self.transform_offsets = {bone: TransformOffset() for bone in EXPECTED_BONES}
        self.T_tracker_chest = np.eye(4)
        self.T_tracker_upperL = np.eye(4)
        self.T_tracker_upperR = np.eye(4)
        self.calibrated = False
        self.calibrated_pub.publish(Bool(data=False))
        self.get_logger().info("Reset calibration to default")

    def _handle_calibration(self, payload: dict) -> None:
        """Handle calibration button presses.

        Args:
            payload: Vive payload dict containing controller input data.
        """
        joy_data = payload.get("controllerInput", {})

        if joy_data.get("rightMenu", 0) == 1 and not self.calibrated:
            self.calibrate(payload)
        if joy_data.get("leftMenu", 0) == 1 and self.calibrated:
            self.reset_calibration()

    def calibrate(self, payload: dict) -> None:
        """Calibrate the VR system based on current controller positions.

        Args:
            payload: The payload dictionary containing controller data.
        """
        _, q_left = self._get_frame_pose("LeftElbow")
        _, q_right = self._get_frame_pose("RightElbow")
        _, q_chest = self._get_frame_pose("Waist")
        left = pose_to_matrix([0, 0, 0], q_left)
        right = pose_to_matrix([0, 0, 0], q_right)
        chest = pose_to_matrix([0, 0, 0], q_chest)

        self.T_tracker_chest = invert(chest)
        self.T_tracker_upperL = invert(left)
        self.T_tracker_upperR = invert(right)

        for bone in self.transform_offsets:
            offset = self.transform_offsets[bone].position
            offset.x = 0.0
            offset.y = 0.0
            offset.z = self.zero_offset[bone] - self._frames[bone].transform.translation.z

        hmd_z = self._frames["HMD"].transform.translation.z
        r_controller_z = self._frames["RightController"].transform.translation.z
        l_controller_z = self._frames["LeftController"].transform.translation.z

        right_arm_length = abs(hmd_z - r_controller_z)
        left_arm_length = abs(hmd_z - l_controller_z)
        avg_arm_length = 0.5 * (right_arm_length + left_arm_length)

        if avg_arm_length > 1e-6:
            self.robot_scale = self.robot_arm_length / avg_arm_length
        else:
            self.robot_scale = 1.0
            self.get_logger().warn(
                "Calibration arm length is too small, fallback robot_scale to 1.0"
            )

        self.calibrated = True
        self.calibrated_pub.publish(Bool(data=True))

        self.get_logger().info(
            f"Calibrated: robot_scale={self.robot_scale:.3f}, "
            f"offsets={[(k, v.position.z) for k, v in self.transform_offsets.items()]}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ViveMocap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting vive_mocap down...")
    finally:
        if getattr(node, "sock", None) is not None:
            node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
