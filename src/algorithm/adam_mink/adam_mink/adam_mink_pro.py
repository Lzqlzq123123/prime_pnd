#!/usr/bin/env python3
"""Noitom-based Adam Pro robot inverse kinematics node using Mink solver."""

import math

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation

from adam_mink.adam_mink_base import AdamMinkBase
from adam_mink.constants import ROOT_POSE_NUM
from adam_mink.thumb_calibrator import (
    CALIBRATION_TARGETS,
    DEFAULT_CALIBRATE_DURATION_SEC,
    ThumbCalibrator,
)
from adam_mink.utils import quat_mul_single

MocapData = dict[str, tuple[np.ndarray, np.ndarray]]
# Constants for finger mapping
DEFAULT_FINGER_LOWER_LIMIT = 0.0
DEFAULT_FINGER_UPPER_LIMIT_THUMB_1 = 1.2
DEFAULT_FINGER_UPPER_LIMIT_THUMB_2 = 0.5
DEFAULT_FINGER_UPPER_LIMIT_OTHER = math.pi
DEFAULT_FINGER_MAPPING_SCALE = 1000.0


class AdamMinkProNode(AdamMinkBase):
    """Noitom-based Adam Pro robot IK node without joystick control."""

    def __init__(self) -> None:
        """Initialize the Pro node."""
        super().__init__("adam_mink_pro")
        self.declare_parameter("noitom_hand_calibrate", False)
        self.declare_parameter("calibrate", False)
        self.declare_parameter("calibrate_duration_sec", DEFAULT_CALIBRATE_DURATION_SEC)
        self.add_on_set_parameters_callback(self.param_cb)

        self._calibrator = ThumbCalibrator(
            default_limits={
                "hand_thumb_1_Left": [
                    DEFAULT_FINGER_LOWER_LIMIT,
                    DEFAULT_FINGER_UPPER_LIMIT_THUMB_1,
                ],
                "hand_thumb_2_Left": [
                    DEFAULT_FINGER_LOWER_LIMIT,
                    DEFAULT_FINGER_UPPER_LIMIT_THUMB_2,
                ],
                "hand_thumb_1_Right": [
                    DEFAULT_FINGER_LOWER_LIMIT,
                    DEFAULT_FINGER_UPPER_LIMIT_THUMB_1,
                ],
                "hand_thumb_2_Right": [
                    DEFAULT_FINGER_LOWER_LIMIT,
                    DEFAULT_FINGER_UPPER_LIMIT_THUMB_2,
                ],
            },
            logger=self.get_logger(),
        )

        self.finger_handle_param = self._create_finger_handle_params()
        self.rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
        self.rotation_matrix_T = self.rotation_matrix.T
        self.rotation_quat = Rotation.from_matrix(self.rotation_matrix).as_quat(scalar_first=True)
        self.calibrated = True
        self.get_logger().info("Adam Mink Pro IK node initialized.")

    # ------------------------------------------------------------------
    # ROS parameter callback
    # ------------------------------------------------------------------

    def param_cb(self, params):
        for param in params:
            if param.name == "calibrate":
                if param.value:
                    duration = float(self.get_parameter("calibrate_duration_sec").value)
                    self._calibrator.start(duration)
            elif param.name == "calibrate_duration_sec" and param.value <= 0:
                return SetParametersResult(
                    successful=False,
                    reason="calibrate_duration_sec must be > 0",
                )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Calibration sampling (called every control-loop tick)
    # ------------------------------------------------------------------

    def _read_joint_sums(self) -> dict[str, float]:
        """Build {target_name: summed_joint_angle} for calibration targets."""
        sums: dict[str, float] = {}
        for target_name, joint_names in CALIBRATION_TARGETS.items():
            total = 0.0
            for name in joint_names:
                idx = self.all_joint.get(f"dof_pos/{name}")
                if idx is not None:
                    total += self.joint_state_msg.position[ROOT_POSE_NUM + idx]
            sums[target_name] = total
        return sums

    def _tick_calibration(self) -> None:
        """Feed one sample to calibrator; rebuild params when done."""
        if not self._calibrator.active:
            return
        if self._calibrator.collect(self._read_joint_sums()):
            self.finger_handle_param = self._create_finger_handle_params()
            self.get_logger().info("Thumb calibration complete, per-hand min/max limits applied")
            self.set_parameters([Parameter("calibrate", type_=Parameter.Type.BOOL, value=False)])

    # ------------------------------------------------------------------
    # Finger handle params
    # ------------------------------------------------------------------

    def _create_finger_handle_params(self) -> list[tuple]:
        """Create finger handle parameter configuration.

        Returns:
            List of tuples containing (finger_idx, joints_idx, lower_limit, upper_limit).
        """
        lim = self._calibrator.limits
        finger_configs = [
            # Left hand
            ("hand_thumb_2_Left", ["L_thumb_MCP_joint1"], *lim["hand_thumb_2_Left"]),
            (
                "hand_thumb_1_Left",
                ["L_thumb_MCP_joint2", "L_thumb_PIP_joint", "L_thumb_DIP_joint"],
                *lim["hand_thumb_1_Left"],
            ),
            (
                "hand_index_Left",
                ["L_index_MCP_joint", "L_index_DIP_joint"],
                DEFAULT_FINGER_LOWER_LIMIT,
                DEFAULT_FINGER_UPPER_LIMIT_OTHER,
            ),
            (
                "hand_middle_Left",
                ["L_middle_MCP_joint", "L_middle_DIP_joint"],
                DEFAULT_FINGER_LOWER_LIMIT,
                DEFAULT_FINGER_UPPER_LIMIT_OTHER,
            ),
            (
                "hand_ring_Left",
                ["L_ring_MCP_joint", "L_ring_DIP_joint"],
                DEFAULT_FINGER_LOWER_LIMIT,
                DEFAULT_FINGER_UPPER_LIMIT_OTHER,
            ),
            (
                "hand_pinky_Left",
                ["L_pinky_MCP_joint", "L_pinky_DIP_joint"],
                DEFAULT_FINGER_LOWER_LIMIT,
                DEFAULT_FINGER_UPPER_LIMIT_OTHER,
            ),
            # Right hand
            ("hand_thumb_2_Right", ["R_thumb_MCP_joint1"], *lim["hand_thumb_2_Right"]),
            (
                "hand_thumb_1_Right",
                ["R_thumb_MCP_joint2", "R_thumb_PIP_joint", "R_thumb_DIP_joint"],
                *lim["hand_thumb_1_Right"],
            ),
            (
                "hand_index_Right",
                ["R_index_MCP_joint", "R_index_DIP_joint"],
                DEFAULT_FINGER_LOWER_LIMIT,
                DEFAULT_FINGER_UPPER_LIMIT_OTHER,
            ),
            (
                "hand_middle_Right",
                ["R_middle_MCP_joint", "R_middle_DIP_joint"],
                DEFAULT_FINGER_LOWER_LIMIT,
                DEFAULT_FINGER_UPPER_LIMIT_OTHER,
            ),
            (
                "hand_ring_Right",
                ["R_ring_MCP_joint", "R_ring_DIP_joint"],
                DEFAULT_FINGER_LOWER_LIMIT,
                DEFAULT_FINGER_UPPER_LIMIT_OTHER,
            ),
            (
                "hand_pinky_Right",
                ["R_pinky_MCP_joint", "R_pinky_DIP_joint"],
                DEFAULT_FINGER_LOWER_LIMIT,
                DEFAULT_FINGER_UPPER_LIMIT_OTHER,
            ),
        ]

        params = []
        for hand_name, joint_names, lower_limit, upper_limit in finger_configs:
            finger_idx = self.all_joint.get(f"dof_pos/{hand_name}")
            joints_idx = [self.all_joint.get(f"dof_pos/{name}") for name in joint_names]
            params.append((finger_idx, joints_idx, lower_limit, upper_limit))
        return params

    # ------------------------------------------------------------------
    # Bone frames
    # ------------------------------------------------------------------

    def get_bone_frames(self) -> list[str]:
        """Get the list of bone frame names to track for Noitom."""
        return [
            "Hips",
            "RightUpLeg",
            "RightLeg",
            "RightFoot",
            "LeftUpLeg",
            "LeftLeg",
            "LeftFoot",
            # "Spine",
            # "Spine1",
            "Spine2",
            # "Neck",
            # "Neck1",
            "Head",
            # "RightShoulder",
            "RightArm",
            "RightForeArm",
            "RightHand",
            "RightHandThumb1",
            "RightHandThumb2",
            "RightHandThumb3",
            # "RightInHandIndex",
            "RightHandIndex1",
            "RightHandIndex2",
            # "RightHandIndex3",
            # "RightInHandMiddle",
            "RightHandMiddle1",
            "RightHandMiddle2",
            # "RightHandMiddle3",
            # "RightInHandRing",
            "RightHandRing1",
            "RightHandRing2",
            # "RightHandRing3",
            # "RightInHandPinky",
            "RightHandPinky1",
            "RightHandPinky2",
            # "RightHandPinky3",
            # "LeftShoulder",
            "LeftArm",
            "LeftForeArm",
            "LeftHand",
            "LeftHandThumb1",
            "LeftHandThumb2",
            "LeftHandThumb3",
            # "LeftInHandIndex",
            "LeftHandIndex1",
            "LeftHandIndex2",
            # "LeftHandIndex3",
            # "LeftInHandMiddle",
            "LeftHandMiddle1",
            "LeftHandMiddle2",
            # "LeftHandMiddle3",
            # "LeftInHandRing",
            "LeftHandRing1",
            "LeftHandRing2",
            # "LeftHandRing3",
            # "LeftInHandPinky",
            "LeftHandPinky1",
            "LeftHandPinky2",
            # "LeftHandPinky3",
        ]

    # ------------------------------------------------------------------
    # Mocap / joint-state helpers
    # ------------------------------------------------------------------

    def offset_mocap_data(self, data: MocapData) -> None:
        for name, (pos, rot) in data.items():
            rotated_pos = pos @ self.rotation_matrix_T
            rotated_quat = quat_mul_single(self.rotation_quat, rot)
            data[name] = (rotated_pos, rotated_quat)

        super().offset_mocap_data(data)

    def update_joint_states(self) -> None:
        """Update finger joint states based on mapped joint angles."""
        self._tick_calibration()

        for (
            finger_idx,
            joints_idx,
            lower_limit,
            upper_limit,
        ) in self.finger_handle_param:
            joints_radian = 0.0
            for joint_idx in joints_idx:
                if joint_idx is not None:
                    joints_radian += self.joint_state_msg.position[ROOT_POSE_NUM + joint_idx]
            if finger_idx is not None:
                self.joint_state_msg.position[ROOT_POSE_NUM + finger_idx] = self.convert_finger(
                    joints_radian, lower_limit, upper_limit
                )

    def convert_finger(self, radian: float, lower_limit: float, upper_limit: float) -> float:
        """Convert joint angle (radian) to finger control value.

        Args:
            radian: Joint angle in radians.
            lower_limit: Lower limit for the joint angle.
            upper_limit: Upper limit for the joint angle.

        Returns:
            Finger control value mapped to [0, 1000] range.
        """
        if radian < lower_limit:
            radian = lower_limit
        elif radian > upper_limit:
            radian = upper_limit

        if upper_limit == lower_limit:
            return DEFAULT_FINGER_MAPPING_SCALE

        mapped_value = DEFAULT_FINGER_MAPPING_SCALE - int(
            ((radian - lower_limit) / (upper_limit - lower_limit)) * DEFAULT_FINGER_MAPPING_SCALE
        )
        return float(mapped_value)


def main(args=None) -> None:
    """Main entry point for the Pro node."""
    rclpy.init(args=args)

    node = AdamMinkProNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
