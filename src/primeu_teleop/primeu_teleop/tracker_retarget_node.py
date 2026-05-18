#!/usr/bin/env python3
"""PrimeU tracker-based teleoperation node.

Subscribes to libsurvive_ros2 TF frames, solves Mink IK against the
primeu_description MuJoCo model, and publishes /joint_states.

Design:
- Subclass of AdamMinkBase (from adam_mink package) so we reuse the
  scaling, offset, IK-solving and joint-publishing pipeline.
- We override get_bone_frames() to return tracker frame IDs declared in
  the YAML config.
"""

from __future__ import annotations

import sys
import re
import time
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import mujoco as mj
import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
import mink
from mink.exceptions import NoSolutionFound
from scipy.spatial.transform import Rotation
from adam_mink.adam_mink_base import AdamMinkBase
from adam_mink.constants import (
    DEFAULT_MOCAP_STALE_TIMEOUT,
    DEFAULT_TIMER_PERIOD,
    ROOT_POSE_NUM,
)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

if TYPE_CHECKING:
    from adam_mink.adam_mink_base import IkConfig
else:
    IkConfig = Any


PLACEHOLDER_PREFIX = "TRACKER_"
JOINT_DOF_WIDTH = {
    mj.mjtJoint.mjJNT_FREE: 6,
    mj.mjtJoint.mjJNT_BALL: 3,
    mj.mjtJoint.mjJNT_SLIDE: 1,
    mj.mjtJoint.mjJNT_HINGE: 1,
}
WAIST_LINK_NAME = "waist_yaw_link"
CHEST_LINK_NAME = "chest_link"
LEFT_WRIST_LINK_NAME = "left_wrist_yaw_link"
RIGHT_WRIST_LINK_NAME = "right_wrist_yaw_link"
RELATIVE_CONTROL_LINKS = (
    WAIST_LINK_NAME,
    CHEST_LINK_NAME,
    LEFT_WRIST_LINK_NAME,
    RIGHT_WRIST_LINK_NAME,
)
RELATIVE_CONTROL_IK_TARGET_LINKS = (
    WAIST_LINK_NAME,
    CHEST_LINK_NAME,
    LEFT_WRIST_LINK_NAME,
    RIGHT_WRIST_LINK_NAME,
)
TORSO_RELATIVE_KEY = "torso"
LEFT_ARM_RELATIVE_KEY = "left_arm"
RIGHT_ARM_RELATIVE_KEY = "right_arm"
TORSO_ACTIVE_JOINTS = {
    "waist_yaw_joint",
    "waist_roll_passive_joint",
    "waist_pitch_passive_joint",
}
LEFT_ARM_ACTIVE_JOINTS = {
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
}
RIGHT_ARM_ACTIVE_JOINTS = {
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
}
UPPER_BODY_ACTIVE_JOINTS = (
    TORSO_ACTIVE_JOINTS | LEFT_ARM_ACTIVE_JOINTS | RIGHT_ARM_ACTIVE_JOINTS
)
BALL_TO_EULER_ORDER = "xyz"
YAW_EULER_ORDER = "zyx"
MAX_TARGET_TRANSLATION_STEP = 0.05
MAX_TARGET_ROTATION_STEP_DEG = 10.0
MIN_ARM_WORKSPACE_SCALE = 0.25
MAX_ARM_WORKSPACE_SCALE = 1.0


@dataclass
class RelativeControlAnchors:
    """Neutral tracker/robot poses used for relative teleoperation."""

    tracker_world: dict[str, mink.SE3]
    tracker_relatives: dict[str, mink.SE3]
    robot_world: dict[str, mink.SE3]
    robot_relatives: dict[str, mink.SE3]
    arm_workspace_scales: dict[str, float]


@dataclass
class DirectControlAnchors:
    """Neutral tracker/robot poses used for anchored direct teleoperation."""

    tracker_world: dict[str, mink.SE3]
    robot_world: dict[str, mink.SE3]


@dataclass
class SolveProfile:
    """Per-cycle timing information for basic runtime visibility."""

    total_solve_time: float = 0.0
    cycle_time: float = 0.0


class PrimeUTrackerRetarget(AdamMinkBase):
    """Teleop node: libsurvive trackers -> primeu joint states via Mink IK."""

    def __init__(self) -> None:
        # AdamMinkBase starts the IK thread inside super().__init__(), and that
        # thread dispatches to this subclass's ik_thread_loop() immediately.
        # Initialize any fields touched by the loop before calling super() to
        # avoid attribute races during startup.
        self._last_rate_log_time = 0.0
        self._rate_log_cycles = 0
        self._last_commanded_target_poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}

        # 初始化父类，加载参数和配置
        super().__init__("primeu_tracker_retarget")

        # After-calibration flag: many base stations publish /tf as soon as
        # they are located. We assume TF is ready, so mark calibrated=True
        # right away. If you want a deliberate calibration step, flip this
        # from the `calibrate` node via a service or topic.
        self.calibrated = True
        if self._relative_control_enabled:
            mode_label = "relative 4-tracker upper-body mode"
        elif self._anchored_direct_control_enabled:
            mode_label = "anchored direct tracker mode"
        else:
            mode_label = "direct tracker mode"
        self.get_logger().info(
            "PrimeU tracker retarget node ready in "
            f"{mode_label}. Listening to TF frames: " + ", ".join(self.bone_frames)
        )

    def _load_parameters(self) -> tuple[str, str, bool]:
        """Load base parameters and apply fixed PrimeU upper-body settings."""
        adam_model_path, adam_mink_cfg_path, mujoco_sim = super()._load_parameters()
        self.freeze_non_arm_dofs = True
        self.declare_parameter("split_upper_body_ik", False)
        self.declare_parameter("ik_loop_hz", 20.0)
        self.declare_parameter("ik_dt", 0.0)
        self.declare_parameter("qp_iter_limit", 80)
        self.declare_parameter("use_configuration_limits", True)
        self.declare_parameter("use_velocity_limits", True)
        self.declare_parameter("anchor_targets_to_initial_pose", True)
        self.declare_parameter("publish_tracker_relative_pose", True)
        self.declare_parameter("tracker_translation_scale", 1.0)
        self.split_upper_body_ik = bool(self.get_parameter("split_upper_body_ik").value)
        self.ik_loop_hz = float(self.get_parameter("ik_loop_hz").value)
        self.ik_dt = float(self.get_parameter("ik_dt").value)
        self.qp_iter_limit = int(self.get_parameter("qp_iter_limit").value)
        self.use_configuration_limits = bool(self.get_parameter("use_configuration_limits").value)
        self.use_velocity_limits = bool(self.get_parameter("use_velocity_limits").value)
        self.anchor_targets_to_initial_pose = bool(
            self.get_parameter("anchor_targets_to_initial_pose").value
        )
        self.publish_tracker_relative_pose = bool(
            self.get_parameter("publish_tracker_relative_pose").value
        )
        self.tracker_translation_scale = float(self.get_parameter("tracker_translation_scale").value)
        self._ik_loop_period = 1.0 / self.ik_loop_hz if self.ik_loop_hz > 0.0 else 0.0
        if self.ik_dt < 0.0:
            raise ValueError("ik_dt must be non-negative")
        if self.qp_iter_limit < 0:
            raise ValueError("qp_iter_limit must be non-negative")
        return adam_model_path, adam_mink_cfg_path, mujoco_sim

    @staticmethod
    def _read_ik_entries(cfg_path: str) -> list[dict]:
        """Load raw IK entries from the YAML config."""
        with open(cfg_path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return data.get("ik_cfg", [])

    @staticmethod
    def _is_placeholder_bone(bone_name: str | None) -> bool:
        """Return True when the configured bone name is still a template placeholder."""
        return bool(bone_name) and bone_name.startswith(PLACEHOLDER_PREFIX)

    def get_bone_frames(self) -> list[str]:
        """Return the list of TF child_frame_id names we want to track.

        Pulled from the IK config's bone_name fields so the YAML is the
        single source of truth.
        """
        # self.adam_mink_cfg is populated by the parent's _load_config(),
        # but get_bone_frames() is called BEFORE _load_config() in the
        # parent __init__. Re-parse the config path here to get the names.
        cfg_path = self.get_parameter("adam_mink_cfg").value
        bones: list[str] = []
        for entry in self._read_ik_entries(cfg_path):
            bone = entry.get("bone_name")
            # Skip placeholder names so users can leave unused entries in
            # the template without causing TF lookup warnings.
            if bone and not self._is_placeholder_bone(bone):
                # 跳过以 TRACKER_ 开头的占位符名称,即配置中写了bone_name对应的tracker id后才会被加入bones列表
                bones.append(bone)

        if not bones:
            self.get_logger().warning(
                "No tracker bone_name entries found in config; "
                "edit primeu_libsurvive_mink_cfg.yaml and replace the "
                "TRACKER_* placeholders with real tracker IDs."
            )
        return bones

    def _load_config(self, adam_mink_cfg_path: str) -> None:
        """Load config and drop placeholder IK entries before tasks are created."""
        super()._load_config(adam_mink_cfg_path)

        placeholder_entries = [
            cfg for cfg in self.adam_mink_cfg.ik_cfg if self._is_placeholder_bone(cfg.bone_name)
        ]
        if placeholder_entries:
            valid_ik_cfg = [
                cfg for cfg in self.adam_mink_cfg.ik_cfg if not self._is_placeholder_bone(cfg.bone_name)
            ]
            if not valid_ik_cfg:
                cfg_label = Path(adam_mink_cfg_path).name
                raise ValueError(
                    f"IK config '{cfg_label}' contains only TRACKER_* placeholders; "
                    "replace at least one bone_name with a real tracker ID."
                )

            skipped_bones = ", ".join(cfg.bone_name for cfg in placeholder_entries)
            self.get_logger().warning(
                "Skipping placeholder IK entries with no real tracker TF: " + skipped_bones
            )

            self.adam_mink_cfg.ik_cfg = valid_ik_cfg
            self.bone_name_to_cfg = {cfg.bone_name: cfg for cfg in self.adam_mink_cfg.ik_cfg}
            self._rot_offset_quats = {
                cfg.bone_name: self._rot_offset_quats[cfg.bone_name]
                for cfg in self.adam_mink_cfg.ik_cfg
                if cfg.bone_name in self._rot_offset_quats
            }
            self._pos_offsets = {
                cfg.bone_name: self._pos_offsets[cfg.bone_name]
                for cfg in self.adam_mink_cfg.ik_cfg
                if cfg.bone_name in self._pos_offsets
            }
            self.mocap_data = self._initialize_mocap_data()
        self._configure_relative_control()

    def _initialize_ik_solver(self) -> None:
        """Initialize unified IK tasks and freeze non-teleop DOFs for validation."""
        super()._initialize_ik_solver()
        self._active_upper_body_joint_names = self._configured_active_joint_names()
        self._frozen_dof_constraint = self._create_upper_body_freeze_constraint()
        self._solve_profile = SolveProfile()
        self._tracker_relative_pose_publishers: dict[str, Any] = {}
        if self.split_upper_body_ik:
            self.get_logger().warning(
                "Parameter 'split_upper_body_ik' is deprecated and ignored; "
                "PrimeU tracker retarget always uses a single QP with multiple FrameTasks."
            )
        self.get_logger().info(
            "Unified upper-body IK enabled with "
            f"{len(self.tasks)} FrameTasks and velocity limits "
            f"{'on' if self.use_velocity_limits else 'off'}, "
            f"translation_scale={self.tracker_translation_scale:.2f}, "
            f"ik_dt={self._ik_solve_dt():.4f}s."
        )
        if self._relative_control_enabled:
            target_links = ", ".join(cfg.adam_link_name for cfg in self._task_ik_cfg)
            self.get_logger().info(
                "Relative 4-tracker mode constrains torso from waist/chest relative motion; "
                f"IK targets: {target_links}."
            )
        if TORSO_ACTIVE_JOINTS.issubset(self._active_upper_body_joint_names):
            self.get_logger().info(
                "Torso tracker target detected; waist joints remain available to the unified QP."
            )
        else:
            self.get_logger().info(
                "No torso tracker target configured; waist joints are frozen in the unified QP."
            )
        if self.publish_tracker_relative_pose:
            for cfg in self.adam_mink_cfg.ik_cfg:
                topic_name = self._tracker_relative_topic_name(cfg.bone_name)
                self._tracker_relative_pose_publishers[cfg.bone_name] = self.create_publisher(
                    PoseStamped, topic_name, 10
                )

    def _initialize_joint_mappings(self) -> None:
        """Build JointState names from MuJoCo joints so they match the PrimeU URDF."""
        
        # 遍历MuJoCo模型中的所有关节，为 sensor_msgs/msg/JointState 消息构建关节名称列表
        self.robot_motor_names = {}
        for i in range(self.model.nu):
            motor_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i)
            self.robot_motor_names[f"dof_pos/{motor_name}"] = i

        self.finger_joint_num = 0
        self._joint_state_specs: list[tuple[int, int, int]] = []
        joint_state_names = []

        for joint_id in range(self.model.njnt):
            joint_name = self.model.joint(joint_id).name
            joint_type = self.model.jnt_type[joint_id]

            if joint_type == mj.mjtJoint.mjJNT_FREE:
                continue

            if joint_type == mj.mjtJoint.mjJNT_BALL:
                for axis_index, axis_name in enumerate(BALL_TO_EULER_ORDER):
                    joint_state_names.append(f"dof_pos/{joint_name}_{axis_name}")
                    self._joint_state_specs.append((joint_id, joint_type, axis_index))
                continue

            joint_state_names.append(f"dof_pos/{joint_name}")
            self._joint_state_specs.append((joint_id, joint_type, -1))

        self.all_joint = {name: i for i, name in enumerate(joint_state_names)}
        self.joint_state_msg = JointState()
        self._joint_state_names = (
            [f"root_pos/{axis}" for axis in ("x", "y", "z")]
            + [f"root_quat/{r}" for r in ("w", "x", "y", "z")]
            + joint_state_names
        )
        self.joint_state_msg.name = self._joint_state_names
        self.joint_state_msg.position = [0.0] * len(self.joint_state_msg.name)
        self._qpos_size = self.configuration.data.qpos.size
        self._last_valid_qpos = np.asarray(self.configuration.data.qpos, dtype=np.float64).copy()
        self._last_valid_joint_positions = [0.0] * len(self.joint_state_msg.name)
        self._last_publish_error_time = 0.0

    def _create_upper_body_freeze_constraint(self) -> mink.DofFreezingTask | None:
        """Freeze the free base and optionally other non-teleop DOFs."""

        frozen_dof_indices = list(range(6))  # Freeze the free base.
        if self.freeze_non_arm_dofs:
            for joint_id in range(self.model.njnt):
                joint_name = self.model.joint(joint_id).name
                if joint_name in self._active_upper_body_joint_names:
                    continue

                dof_start = self.model.jnt_dofadr[joint_id]
                if dof_start < 0:
                    continue
                dof_width = JOINT_DOF_WIDTH.get(self.model.jnt_type[joint_id], 0)
                dof_end = min(dof_start + dof_width, self.model.nv)
                frozen_dof_indices.extend(range(dof_start, dof_end))

        frozen_dof_indices = sorted(set(idx for idx in frozen_dof_indices if 0 <= idx < self.model.nv))
        if not frozen_dof_indices:
            return None

        if self.freeze_non_arm_dofs:
            self.get_logger().info(
                "Upper-body mode enabled: free base and non-teleop DOFs are frozen "
                f"({len(frozen_dof_indices)} constrained velocities)"
            )
        else:
            self.get_logger().info(
                "Upper-body mode enabled: free base is frozen while teleop joints remain free "
                f"({len(frozen_dof_indices)} constrained velocities)"
            )
        return mink.DofFreezingTask(model=self.model, dof_indices=frozen_dof_indices)

    def _configured_active_joint_names(self) -> set[str]:
        """Return the upper-body joints that should remain free for the current task set."""
        active_joint_names = set(LEFT_ARM_ACTIVE_JOINTS | RIGHT_ARM_ACTIVE_JOINTS)
        configured_links = {cfg.adam_link_name for cfg in self.adam_mink_cfg.ik_cfg}
        if WAIST_LINK_NAME in configured_links or CHEST_LINK_NAME in configured_links:
            active_joint_names |= TORSO_ACTIVE_JOINTS
        return active_joint_names

    def _create_frame_task(self, cfg: IkConfig) -> mink.FrameTask:
        """Create a single Mink FrameTask from one IK config entry."""
        return mink.FrameTask(
            frame_name=cfg.adam_link_name,
            frame_type="body",
            position_cost=cfg.position_cost,
            orientation_cost=cfg.orientation_cost,
            lm_damping=1.0,
        )

    def _target_ik_configs(self) -> list[IkConfig]:
        """Return config entries that should become actual Mink FrameTasks."""
        if not getattr(self, "_relative_control_enabled", False):
            return list(self.adam_mink_cfg.ik_cfg)

        target_links = set(RELATIVE_CONTROL_IK_TARGET_LINKS)
        return [
            cfg for cfg in self.adam_mink_cfg.ik_cfg if cfg.adam_link_name in target_links
        ]

    def _create_ik_tasks(self) -> list[mink.FrameTask]:
        """Create IK tasks; in 4-tracker mode chest is a reference frame only."""
        self._task_ik_cfg = self._target_ik_configs()
        return [self._create_frame_task(cfg) for cfg in self._task_ik_cfg]

    def _create_ik_limits(
        self,
    ) -> list[mink.ConfigurationLimit | mink.CollisionAvoidanceLimit | mink.VelocityLimit]:
        """Allow runtime diagnostics with or without velocity limits."""
        limits = super()._create_ik_limits()
        if not self.use_configuration_limits:
            limits = [limit for limit in limits if not isinstance(limit, mink.ConfigurationLimit)]
        if self.use_velocity_limits:
            return limits
        return [limit for limit in limits if not isinstance(limit, mink.VelocityLimit)]

    @staticmethod
    def _normalize_quaternion(quat: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        """Return a normalized scalar-first quaternion."""
        quat_arr = np.asarray(quat, dtype=np.float64).copy()
        quat_norm = np.linalg.norm(quat_arr)
        if quat_norm > 0.0:
            quat_arr /= quat_norm
        else:
            quat_arr = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return quat_arr

    def _scale_translation_only(self, transform: mink.SE3) -> mink.SE3:
        """Scale the translational part of a pose delta while preserving rotation."""
        scale = float(getattr(self, "tracker_translation_scale", 1.0))
        scaled_translation = np.asarray(transform.translation(), dtype=np.float64) * scale
        return mink.SE3.from_rotation_and_translation(
            mink.SO3(np.asarray(transform.rotation().parameters(), dtype=np.float64)),
            scaled_translation,
        )

    @staticmethod
    def _rotation_only(transform: mink.SE3) -> mink.SE3:
        """Keep only the rotational part of a relative motion."""
        return mink.SE3.from_rotation_and_translation(
            mink.SO3(np.asarray(transform.rotation().parameters(), dtype=np.float64)),
            np.zeros(3),
        )

    def _map_tracker_chest_delta_to_robot_chest(
        self,
        anchors: RelativeControlAnchors,
        tracker_chest_delta: np.ndarray,
    ) -> np.ndarray:
        """Map a translation delta from neutral tracker-chest axes to robot-chest axes."""
        tracker_chest_rot = Rotation.from_quat(
            self._normalize_quaternion(
                anchors.tracker_world[CHEST_LINK_NAME].rotation().parameters()
            ),
            scalar_first=True,
        )
        robot_chest_rot = Rotation.from_quat(
            self._normalize_quaternion(
                anchors.robot_world[CHEST_LINK_NAME].rotation().parameters()
            ),
            scalar_first=True,
        )
        world_delta = tracker_chest_rot.apply(np.asarray(tracker_chest_delta, dtype=np.float64))
        robot_chest_delta = robot_chest_rot.inv().apply(world_delta)
        return robot_chest_delta * float(getattr(self, "tracker_translation_scale", 1.0))

    def _tracker_relative_topic_name(self, bone_name: str) -> str:
        """Build a ROS-safe topic name for a tracker-relative pose diagnostic."""
        safe_bone_name = re.sub(r"[^A-Za-z0-9_]", "_", bone_name)
        return f"/{self.get_name()}/tracker_relative_pose/{safe_bone_name}"

    def _reset_control_anchors(self, reason: str | None = None) -> None:
        """Drop cached anchor state so motion resumes from a fresh neutral snapshot."""
        had_anchor_state = (
            self._relative_control_anchors is not None
            or self._direct_control_anchors is not None
            or bool(self._last_commanded_target_poses)
        )
        self._relative_control_anchors = None
        self._direct_control_anchors = None
        self._last_commanded_target_poses.clear()
        if had_anchor_state and reason:
            self.get_logger().warning(reason)

    @staticmethod
    def _limit_translation_step(
        previous_position: np.ndarray,
        target_position: np.ndarray,
        max_step: float,
    ) -> np.ndarray:
        """Clamp a target translation step to keep commanded motion continuous."""
        if max_step <= 0.0:
            return target_position

        delta = target_position - previous_position
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm <= max_step or delta_norm == 0.0:
            return target_position
        return previous_position + delta * (max_step / delta_norm)

    @classmethod
    def _limit_rotation_step(
        cls,
        previous_quaternion: np.ndarray,
        target_quaternion: np.ndarray,
        max_step_deg: float,
    ) -> np.ndarray:
        """Clamp a target rotation step along the shortest SO(3) arc."""
        target_quaternion = cls._normalize_quaternion(target_quaternion)
        if max_step_deg <= 0.0:
            return target_quaternion

        previous_rotation = Rotation.from_quat(
            cls._normalize_quaternion(previous_quaternion), scalar_first=True
        )
        target_rotation = Rotation.from_quat(target_quaternion, scalar_first=True)
        delta_rotation = previous_rotation.inv() * target_rotation
        delta_angle = float(delta_rotation.magnitude())
        max_step_rad = float(np.deg2rad(max_step_deg))
        if delta_angle <= max_step_rad or delta_angle == 0.0:
            return target_quaternion

        limited_delta = Rotation.from_rotvec(
            delta_rotation.as_rotvec() * (max_step_rad / delta_angle)
        )
        return np.asarray((previous_rotation * limited_delta).as_quat(scalar_first=True))

    def _limit_target_transform_step(self, bone_name: str, transform: mink.SE3) -> mink.SE3:
        """Bound per-cycle target motion to avoid large stale-recovery jumps."""
        position, quaternion = self._pose_from_se3(transform)
        quaternion = self._normalize_quaternion(quaternion)
        previous_pose = self._last_commanded_target_poses.get(bone_name)
        if previous_pose is not None:
            position = self._limit_translation_step(
                previous_pose[0], position, MAX_TARGET_TRANSLATION_STEP
            )
            quaternion = self._limit_rotation_step(
                previous_pose[1], quaternion, MAX_TARGET_ROTATION_STEP_DEG
            )

        self._last_commanded_target_poses[bone_name] = (position.copy(), quaternion.copy())
        return mink.SE3.from_rotation_and_translation(mink.SO3(quaternion), position)

    def _update_ik_targets(self) -> bool:
        """Update unified FrameTask targets from the current adjusted mocap data."""
        task_ik_cfg = getattr(self, "_task_ik_cfg", self.adam_mink_cfg.ik_cfg)
        for task, cfg in zip(self.tasks, task_ik_cfg, strict=True):
            mocap_item = self.mocap_data_adjusted.get(cfg.bone_name)
            if mocap_item is None:
                self.get_logger().warning(f"Bone '{cfg.bone_name}' not found in mocap_data")
                continue

            pos = np.asarray(mocap_item[0], dtype=np.float64).copy()
            rot = self._normalize_quaternion(mocap_item[1])
            task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
        return True

    def _solve_ik(self) -> None:
        """Solve one unified upper-body IK problem with multiple task targets."""
        dt = self._ik_solve_dt()
        num_iter = 0
        constraints = (
            [self._frozen_dof_constraint] if self._frozen_dof_constraint is not None else None
        )
        solve_start_time = time.perf_counter()

        while num_iter < self.ik_iter_max:
            solver_kwargs = self._solver_kwargs()
            try:
                with self._configuration_lock:
                    vel = mink.solve_ik(
                        configuration=self.configuration,
                        tasks=self.tasks,
                        dt=dt,
                        solver=self.ik_solver,
                        damping=self.ik_damping,
                        limits=self.limits,
                        constraints=constraints,
                        **solver_kwargs,
                    )
            except NoSolutionFound:
                self.get_logger().warning(
                    f"IK solver '{self.ik_solver}' did not find a solution within "
                    f"qp_iter_limit={self.qp_iter_limit}; keeping previous joint state."
                )
                break

            with self._configuration_lock:
                self.configuration.integrate_inplace(vel, dt)
                if not np.all(np.isfinite(self.configuration.data.qpos)):
                    self._restore_last_valid_configuration(
                        "IK integration produced non-finite qpos; restoring previous valid state."
                    )
                    break

                self._last_valid_qpos = np.asarray(
                    self.configuration.data.qpos, dtype=np.float64
                ).copy()
            num_iter += 1
        self._solve_profile.total_solve_time = time.perf_counter() - solve_start_time

    def _restore_last_valid_configuration(self, reason: str) -> None:
        """Recover from invalid IK state without killing the ROS node."""
        self._print_publish_error(reason)
        last_valid_qpos = getattr(self, "_last_valid_qpos", None)
        if last_valid_qpos is not None and np.all(np.isfinite(last_valid_qpos)):
            with self._configuration_lock:
                self.configuration.update(last_valid_qpos.copy())

    def _solver_kwargs(self) -> dict[str, int]:
        """Return backend-specific QP limits for real-time use."""
        if self.ik_solver == "daqp" and self.qp_iter_limit > 0:
            return {"iter_limit": self.qp_iter_limit}
        return {}

    def _ik_solve_dt(self) -> float:
        """Return the IK integration step used by velocity limits and integration."""
        if self.ik_dt > 0.0:
            return self.ik_dt
        if self._ik_loop_period > 0.0:
            return self._ik_loop_period
        return float(self.configuration.model.opt.timestep)

    def _publish_joint_states(self) -> None:
        """Publish JointState values with names that match the PrimeU URDF."""
        
        # 获取最新的关节位置
        with self._configuration_lock:
            qpos = np.asarray(self.configuration.data.qpos, dtype=np.float64).copy()
        if not np.all(np.isfinite(qpos)):
            self._restore_last_valid_configuration(
                "Skipping invalid JointState publish because qpos contains non-finite values."
            )
            with self._configuration_lock:
                qpos = np.asarray(self.configuration.data.qpos, dtype=np.float64).copy()

        positions = [0.0] * len(self._joint_state_names)
        positions[:ROOT_POSE_NUM] = qpos[:ROOT_POSE_NUM].tolist()

        # 对于球状关节，它将MuJoCo中的四元数（quaternion）姿态转换为欧拉角（euler angles），然后填充到 JointState 消息对应的位置。
        # 对于其他关节（如铰链或滑块），直接填充 qpos 值。
        ball_euler_cache: dict[int, np.ndarray] = {}
        for idx, (joint_id, joint_type, axis_index) in enumerate(self._joint_state_specs, start=ROOT_POSE_NUM):
            qpos_addr = self.model.jnt_qposadr[joint_id]
            if joint_type == mj.mjtJoint.mjJNT_BALL:
                if joint_id not in ball_euler_cache:
                    quat = qpos[qpos_addr : qpos_addr + 4].copy()
                    quat_norm = np.linalg.norm(quat)
                    if quat_norm > 0.0:
                        quat /= quat_norm
                    else:
                        quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore", UserWarning)
                        ball_euler_cache[joint_id] = Rotation.from_quat(
                            quat, scalar_first=True
                        ).as_euler(BALL_TO_EULER_ORDER, degrees=False)
                positions[idx] = float(ball_euler_cache[joint_id][axis_index])
            else:
                positions[idx] = float(qpos[qpos_addr])

        if not np.all(np.isfinite(positions)):
            self._print_publish_error(
                "Skipping invalid JointState publish because positions contain non-finite values."
            )
            positions = getattr(self, "_last_valid_joint_positions", positions)
        else:
            self._last_valid_joint_positions = positions.copy()

        if len(positions) != len(self._joint_state_names):
            self._print_publish_error(
                f"Skipping invalid JointState publish because name/position length mismatch: "
                f"{len(self._joint_state_names)} names vs {len(positions)} positions."
            )
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_state_names)
        msg.position = [float(value) for value in positions]
        try:
            self.joint_state_pub.publish(msg)
        except RuntimeError as exc:
            self._print_publish_error(f"Failed to publish JointState ({exc}); dropping this frame.")

    def _print_publish_error(self, message: str) -> None:
        """Print publish-path errors without using rosout, which may also be failing."""
        now = time.monotonic()
        if now - getattr(self, "_last_publish_error_time", 0.0) < 1.0:
            return
        self._last_publish_error_time = now
        print(f"[primeu_teleop] {message}", file=sys.stderr, flush=True)

    def _configure_relative_control(self) -> None:
        """Enable 4-tracker relative control when waist/chest/both wrists exist in config."""
        anchor_targets_to_initial_pose = getattr(self, "anchor_targets_to_initial_pose", True)
        self._link_name_to_cfg = {cfg.adam_link_name: cfg for cfg in self.adam_mink_cfg.ik_cfg}
        self._relative_link_to_bone = {
            cfg.adam_link_name: cfg.bone_name
            for cfg in self.adam_mink_cfg.ik_cfg
            if cfg.adam_link_name in RELATIVE_CONTROL_LINKS
        }
        self._relative_control_enabled = all(
            link_name in self._relative_link_to_bone for link_name in RELATIVE_CONTROL_LINKS
        )
        self._relative_control_anchors: RelativeControlAnchors | None = None
        self._anchored_direct_control_enabled = (
            anchor_targets_to_initial_pose
            and not self._relative_control_enabled
            and bool(self.adam_mink_cfg.ik_cfg)
        )
        self._direct_control_anchors: DirectControlAnchors | None = None

        if self._relative_control_enabled:
            self.get_logger().info(
                "Detected waist/chest/left/right tracker config. "
                "Targets will be generated from relative tracker motion after the first neutral snapshot."
            )
            return

        if self._anchored_direct_control_enabled:
            self.get_logger().info(
                "Anchored direct mode enabled. Absolute tracker poses will be converted "
                "into per-link relative motion after the first neutral snapshot."
            )
            return

        configured_links = set(self._relative_link_to_bone)
        if configured_links:
            missing_links = [link for link in RELATIVE_CONTROL_LINKS if link not in configured_links]
            self.get_logger().info(
                "Relative 4-tracker mode not enabled; missing tracker config for: "
                + ", ".join(missing_links)
            )

    @staticmethod
    def _se3_from_pose(pose: tuple[np.ndarray, np.ndarray]) -> mink.SE3:
        """Convert a (position, quaternion) pair into a Mink SE3."""
        pos, rot = pose
        return mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos)

    @staticmethod
    def _pose_from_se3(transform: mink.SE3) -> tuple[np.ndarray, np.ndarray]:
        """Convert a Mink SE3 back into (position, quaternion)."""
        return (
            np.asarray(transform.translation(), dtype=np.float64).copy(),
            np.asarray(transform.rotation().parameters(), dtype=np.float64).copy(),
        )

    @staticmethod
    def _compute_arm_workspace_scale(
        tracker_relative: mink.SE3,
        robot_relative: mink.SE3,
    ) -> float:
        """Scale hand deltas by neutral robot/human reach to avoid workspace magnification."""
        tracker_reach = float(np.linalg.norm(tracker_relative.translation()))
        robot_reach = float(np.linalg.norm(robot_relative.translation()))
        if tracker_reach <= 1e-6 or robot_reach <= 1e-6:
            return 1.0
        return float(
            np.clip(
                robot_reach / tracker_reach,
                MIN_ARM_WORKSPACE_SCALE,
                MAX_ARM_WORKSPACE_SCALE,
            )
        )

    @staticmethod
    def _yaw_only_delta(transform: mink.SE3) -> mink.SE3:
        """Keep only the world-z yaw component of a pose delta."""
        quat = np.asarray(transform.rotation().parameters(), dtype=np.float64)
        yaw = Rotation.from_quat(quat, scalar_first=True).as_euler(
            YAW_EULER_ORDER, degrees=False
        )[0]
        yaw_quat = Rotation.from_euler("z", yaw, degrees=False).as_quat(scalar_first=True)
        return mink.SE3.from_rotation_and_translation(mink.SO3(yaw_quat), np.zeros(3))

    def _current_robot_pose(self, link_name: str) -> mink.SE3:
        """Read the current MuJoCo/Mink pose of a robot body frame."""
        with self._configuration_lock:
            return self.configuration.get_transform_frame_to_world(link_name, "body")

    def _current_tracker_transforms(self, data: dict[str, tuple[np.ndarray, np.ndarray]]) -> dict[str, mink.SE3] | None:
        """Read the current waist/chest/wrist tracker transforms from mocap data."""
        tracker_transforms: dict[str, mink.SE3] = {}
        for link_name, bone_name in self._relative_link_to_bone.items():
            pose = data.get(bone_name)
            if pose is None:
                return None
            tracker_transforms[link_name] = self._se3_from_pose(pose)
        return tracker_transforms

    def _current_direct_tracker_transforms(
        self, data: dict[str, tuple[np.ndarray, np.ndarray]]
    ) -> dict[str, mink.SE3] | None:
        """Read the current tracker transforms for every configured IK target."""
        tracker_transforms: dict[str, mink.SE3] = {}
        for cfg in self.adam_mink_cfg.ik_cfg:
            pose = data.get(cfg.bone_name)
            if pose is None:
                return None
            tracker_transforms[cfg.bone_name] = self._se3_from_pose(pose)
        return tracker_transforms

    def _capture_relative_control_anchors(self, tracker_transforms: dict[str, mink.SE3]) -> None:
        """Capture the neutral tracker and robot poses used as relative-control anchors."""
        robot_world = {
            link_name: self._current_robot_pose(link_name) for link_name in RELATIVE_CONTROL_LINKS
        }
        tracker_relatives = {
            TORSO_RELATIVE_KEY: tracker_transforms[WAIST_LINK_NAME].inverse()
            @ tracker_transforms[CHEST_LINK_NAME],
            LEFT_ARM_RELATIVE_KEY: tracker_transforms[CHEST_LINK_NAME].inverse()
            @ tracker_transforms[LEFT_WRIST_LINK_NAME],
            RIGHT_ARM_RELATIVE_KEY: tracker_transforms[CHEST_LINK_NAME].inverse()
            @ tracker_transforms[RIGHT_WRIST_LINK_NAME],
        }
        robot_relatives = {
            TORSO_RELATIVE_KEY: robot_world[WAIST_LINK_NAME].inverse() @ robot_world[CHEST_LINK_NAME],
            LEFT_ARM_RELATIVE_KEY: robot_world[CHEST_LINK_NAME].inverse()
            @ robot_world[LEFT_WRIST_LINK_NAME],
            RIGHT_ARM_RELATIVE_KEY: robot_world[CHEST_LINK_NAME].inverse()
            @ robot_world[RIGHT_WRIST_LINK_NAME],
        }
        arm_workspace_scales = {
            LEFT_ARM_RELATIVE_KEY: self._compute_arm_workspace_scale(
                tracker_relatives[LEFT_ARM_RELATIVE_KEY],
                robot_relatives[LEFT_ARM_RELATIVE_KEY],
            ),
            RIGHT_ARM_RELATIVE_KEY: self._compute_arm_workspace_scale(
                tracker_relatives[RIGHT_ARM_RELATIVE_KEY],
                robot_relatives[RIGHT_ARM_RELATIVE_KEY],
            ),
        }
        self._relative_control_anchors = RelativeControlAnchors(
            tracker_world={name: pose.copy() for name, pose in tracker_transforms.items()},
            tracker_relatives=tracker_relatives,
            robot_world=robot_world,
            robot_relatives=robot_relatives,
            arm_workspace_scales=arm_workspace_scales,
        )
        self.get_logger().info(
            "Captured neutral tracker snapshot for relative upper-body control. "
            "Arm workspace scale: "
            f"left={arm_workspace_scales[LEFT_ARM_RELATIVE_KEY]:.2f}, "
            f"right={arm_workspace_scales[RIGHT_ARM_RELATIVE_KEY]:.2f}."
        )

    def _capture_direct_control_anchors(self, tracker_transforms: dict[str, mink.SE3]) -> None:
        """Capture the neutral tracker and robot poses used for anchored direct control."""
        robot_world = {
            cfg.bone_name: self._current_robot_pose(cfg.adam_link_name)
            for cfg in self.adam_mink_cfg.ik_cfg
        }
        self._direct_control_anchors = DirectControlAnchors(
            tracker_world={name: pose.copy() for name, pose in tracker_transforms.items()},
            robot_world={name: pose.copy() for name, pose in robot_world.items()},
        )
        self.get_logger().info(
            "Captured neutral tracker snapshot for anchored direct control."
        )

    def _build_relative_targets(self, tracker_transforms: dict[str, mink.SE3]) -> dict[str, mink.SE3]:
        """Generate robot waist/wrist targets from four tracker relative motion."""
        if self._relative_control_anchors is None:
            raise RuntimeError("Relative-control anchors are not initialized")

        anchors = self._relative_control_anchors

        waist_delta_world = anchors.tracker_world[WAIST_LINK_NAME].inverse() @ tracker_transforms[
            WAIST_LINK_NAME
        ]
        waist_target = anchors.robot_world[WAIST_LINK_NAME] @ self._scale_translation_only(
            self._yaw_only_delta(waist_delta_world)
        )

        torso_relative_now = tracker_transforms[WAIST_LINK_NAME].inverse() @ tracker_transforms[
            CHEST_LINK_NAME
        ]
        torso_delta = anchors.tracker_relatives[TORSO_RELATIVE_KEY].inverse() @ torso_relative_now
        chest_target = waist_target @ anchors.robot_relatives[TORSO_RELATIVE_KEY] @ (
            self._scale_translation_only(torso_delta)
        )

        left_relative_now = tracker_transforms[CHEST_LINK_NAME].inverse() @ tracker_transforms[
            LEFT_WRIST_LINK_NAME
        ]
        left_delta = anchors.tracker_relatives[LEFT_ARM_RELATIVE_KEY].inverse() @ left_relative_now
        left_robot_relative = self._build_arm_robot_relative_target(
            anchors,
            LEFT_ARM_RELATIVE_KEY,
            left_relative_now,
            left_delta,
        )
        left_target = chest_target @ left_robot_relative

        right_relative_now = tracker_transforms[CHEST_LINK_NAME].inverse() @ tracker_transforms[
            RIGHT_WRIST_LINK_NAME
        ]
        right_delta = anchors.tracker_relatives[RIGHT_ARM_RELATIVE_KEY].inverse() @ right_relative_now
        right_robot_relative = self._build_arm_robot_relative_target(
            anchors,
            RIGHT_ARM_RELATIVE_KEY,
            right_relative_now,
            right_delta,
        )
        right_target = chest_target @ right_robot_relative

        return {
            WAIST_LINK_NAME: waist_target,
            CHEST_LINK_NAME: chest_target,
            LEFT_WRIST_LINK_NAME: left_target,
            RIGHT_WRIST_LINK_NAME: right_target,
        }

    def _build_arm_robot_relative_target(
        self,
        anchors: RelativeControlAnchors,
        arm_key: str,
        tracker_relative_now: mink.SE3,
        tracker_delta: mink.SE3,
    ) -> mink.SE3:
        """Build a chest-relative wrist target with translation in chest axes."""
        robot_relative_anchor = anchors.robot_relatives[arm_key]
        tracker_relative_anchor = anchors.tracker_relatives[arm_key]
        tracker_chest_delta = (
            np.asarray(tracker_relative_now.translation(), dtype=np.float64)
            - np.asarray(tracker_relative_anchor.translation(), dtype=np.float64)
        )
        robot_chest_delta = self._map_tracker_chest_delta_to_robot_chest(
            anchors,
            tracker_chest_delta,
        )
        arm_workspace_scale = anchors.arm_workspace_scales.get(arm_key, 1.0)
        target_translation = (
            np.asarray(robot_relative_anchor.translation(), dtype=np.float64)
            + robot_chest_delta * arm_workspace_scale
        )
        target_rotation_transform = robot_relative_anchor @ self._rotation_only(tracker_delta)
        return mink.SE3.from_rotation_and_translation(
            mink.SO3(
                np.asarray(target_rotation_transform.rotation().parameters(), dtype=np.float64)
            ),
            target_translation,
        )

    def _build_direct_targets(self, tracker_transforms: dict[str, mink.SE3]) -> dict[str, mink.SE3]:
        """Generate anchored robot link targets from per-tracker relative motion."""
        if self._direct_control_anchors is None:
            raise RuntimeError("Direct-control anchors are not initialized")

        anchors = self._direct_control_anchors
        targets: dict[str, mink.SE3] = {}
        for cfg in self.adam_mink_cfg.ik_cfg:
            tracker_delta = anchors.tracker_world[cfg.bone_name].inverse() @ tracker_transforms[
                cfg.bone_name
            ]
            targets[cfg.bone_name] = anchors.robot_world[cfg.bone_name] @ self._scale_translation_only(
                tracker_delta
            )
        return targets

    def _publish_tracker_relative_pose_messages(
        self, raw_data: dict[str, tuple[np.ndarray, np.ndarray]]
    ) -> None:
        """Publish anchor-relative tracker poses for offline inspection."""
        if not self.publish_tracker_relative_pose:
            return

        if self._relative_control_enabled:
            if self._relative_control_anchors is None:
                return
            current_transforms = self._current_tracker_transforms(raw_data)
            anchors = self._relative_control_anchors.tracker_world
            link_to_bone = self._relative_link_to_bone
        elif self._anchored_direct_control_enabled:
            if self._direct_control_anchors is None:
                return
            current_transforms = self._current_direct_tracker_transforms(raw_data)
            anchors = self._direct_control_anchors.tracker_world
            link_to_bone = {bone_name: bone_name for bone_name in current_transforms or {}}
        else:
            return

        if current_transforms is None:
            return

        for key, current_pose in current_transforms.items():
            anchor_pose = anchors.get(key)
            if anchor_pose is None:
                continue
            relative_pose = anchor_pose.inverse() @ current_pose
            bone_name = link_to_bone[key]
            publisher = self._tracker_relative_pose_publishers.get(bone_name)
            if publisher is None:
                continue

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{self.base_frame}_anchor"
            msg.pose.position.x = float(relative_pose.translation()[0])
            msg.pose.position.y = float(relative_pose.translation()[1])
            msg.pose.position.z = float(relative_pose.translation()[2])
            quat = np.asarray(relative_pose.rotation().parameters(), dtype=np.float64)
            msg.pose.orientation.w = float(quat[0])
            msg.pose.orientation.x = float(quat[1])
            msg.pose.orientation.y = float(quat[2])
            msg.pose.orientation.z = float(quat[3])
            publisher.publish(msg)

    def offset_mocap_data(self, data) -> None:
        """Apply offsets, then convert trackers into anchored robot targets when enabled."""
        super().offset_mocap_data(data)

        if self._relative_control_enabled:
            tracker_transforms = self._current_tracker_transforms(data)
            if tracker_transforms is None:
                return

            if self._relative_control_anchors is None:
                self._capture_relative_control_anchors(tracker_transforms)

            target_transforms = self._build_relative_targets(tracker_transforms)
            for link_name, target in target_transforms.items():
                bone_name = self._relative_link_to_bone[link_name]
                target = self._limit_target_transform_step(bone_name, target)
                data[bone_name] = self._pose_from_se3(target)
            return

        if not self._anchored_direct_control_enabled:
            return

        tracker_transforms = self._current_direct_tracker_transforms(data)
        if tracker_transforms is None:
            return

        if self._direct_control_anchors is None:
            self._capture_direct_control_anchors(tracker_transforms)

        target_transforms = self._build_direct_targets(tracker_transforms)
        for bone_name, target in target_transforms.items():
            target = self._limit_target_transform_step(bone_name, target)
            data[bone_name] = self._pose_from_se3(target)

    def _update_mocap_data(self) -> bool:
        """Reset anchors when tracker TF disappears, otherwise keep base behavior."""
        was_ready = self._mocap_ready
        updated = super()._update_mocap_data()
        if not updated and was_ready:
            self._reset_control_anchors(
                "Tracker TF stream was interrupted; clearing the neutral snapshot so "
                "targets will re-anchor after mocap recovers."
            )
        return updated

    def ik_thread_loop(self) -> None:
        """Run the IK pipeline at a bounded frequency to avoid CPU starvation."""
        self.get_logger().info(
            f"IK thread loop started with target rate {self.ik_loop_hz:.1f} Hz"
        )
        while rclpy.ok():
            cycle_start_time = time.perf_counter()

            if not self.calibrated:
                self._solve_profile.cycle_time = time.perf_counter() - cycle_start_time
                self._maybe_log_loop_rate("waiting_calibration")
                time.sleep(DEFAULT_TIMER_PERIOD)
                continue

            if not self._mocap_ready:
                self._publish_joint_states()
                self._solve_profile.cycle_time = time.perf_counter() - cycle_start_time
                self._maybe_log_loop_rate("waiting_mocap")
                time.sleep(DEFAULT_TIMER_PERIOD)
                continue

            if self._last_mocap_update_time is None:
                self._publish_joint_states()
                self._solve_profile.cycle_time = time.perf_counter() - cycle_start_time
                self._maybe_log_loop_rate("waiting_first_mocap")
                time.sleep(DEFAULT_TIMER_PERIOD)
                continue

            if time.time() - self._last_mocap_update_time > DEFAULT_MOCAP_STALE_TIMEOUT:
                self._mocap_ready = False
                self._reset_control_anchors(
                    "Mocap data went stale; clearing the neutral snapshot so the next "
                    "valid tracker frame captures a fresh anchor."
                )
                self._warn_tf_status(
                    f"Mocap data is stale for more than {DEFAULT_MOCAP_STALE_TIMEOUT:.2f}s; waiting for tracker TF to recover."
                )
                self._publish_joint_states()
                self._solve_profile.cycle_time = time.perf_counter() - cycle_start_time
                self._maybe_log_loop_rate("stale_mocap")
                time.sleep(DEFAULT_TIMER_PERIOD)
                continue

            with self._data_lock:
                mocap_data_copy = self.mocap_data.copy()
            raw_mocap_data = {
                name: (pos.copy(), rot.copy())
                for name, (pos, rot) in mocap_data_copy.items()
            }
            if self.adam_mink_cfg.human_scale_table:
                self.scale_mocap_data(mocap_data_copy)
            self.offset_mocap_data(mocap_data_copy)
            with self._data_lock:
                self.mocap_data_adjusted = mocap_data_copy
            self._update_ik_targets()
            self._solve_ik()
            self._publish_joint_states()
            self._publish_tracker_relative_pose_messages(raw_mocap_data)

            self._solve_profile.cycle_time = time.perf_counter() - cycle_start_time
            self._maybe_log_loop_rate("running")

            if self._ik_loop_period > 0.0:
                remaining = self._ik_loop_period - (time.perf_counter() - cycle_start_time)
                if remaining > 0.0:
                    time.sleep(remaining)

        self.get_logger().info("IK thread loop ended")

    def _maybe_log_loop_rate(self, status: str) -> None:
        """Log the real IK loop rate with negligible per-cycle overhead."""
        self._rate_log_cycles += 1
        now = time.perf_counter()
        if self._last_rate_log_time == 0.0:
            self._last_rate_log_time = now
            self._rate_log_cycles = 0
            return
        elapsed = now - self._last_rate_log_time
        if elapsed < 2.0:
            return
        rate = self._rate_log_cycles / elapsed
        self.get_logger().info(
            f"IK loop status={status}, actual rate: {rate:.1f} Hz, "
            f"last_cycle={self._solve_profile.cycle_time * 1000.0:.1f}ms, "
            f"solve={self._solve_profile.total_solve_time * 1000.0:.1f}ms"
        )
        self._last_rate_log_time = now
        self._rate_log_cycles = 0


def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        node = PrimeUTrackerRetarget()
    except Exception as e:
        print(f"[primeu_teleop] Failed to start node: {e}", file=sys.stderr)
        rclpy.shutdown()
        raise

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
