from __future__ import annotations

import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest
import numpy as np
import mink
from scipy.spatial.transform import Rotation

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

fake_rclpy = types.ModuleType("rclpy")
fake_rclpy.init = lambda *args, **kwargs: None
fake_rclpy.shutdown = lambda *args, **kwargs: None
fake_rclpy.ok = lambda: False
sys.modules.setdefault("rclpy", fake_rclpy)

fake_rclpy_executors = types.ModuleType("rclpy.executors")


class FakeMultiThreadedExecutor:
    def add_node(self, node) -> None:
        pass

    def spin(self) -> None:
        pass


fake_rclpy_executors.MultiThreadedExecutor = FakeMultiThreadedExecutor
sys.modules.setdefault("rclpy.executors", fake_rclpy_executors)

fake_adam_mink_pkg = types.ModuleType("adam_mink")
sys.modules.setdefault("adam_mink", fake_adam_mink_pkg)

fake_adam_mink_base = types.ModuleType("adam_mink.adam_mink_base")
fake_adam_mink_constants = types.ModuleType("adam_mink.constants")
fake_adam_mink_constants.ROOT_POSE_NUM = 7
fake_adam_mink_constants.DEFAULT_MOCAP_STALE_TIMEOUT = 0.5
fake_adam_mink_constants.DEFAULT_SOLVE_WARN_THRESHOLD = 0.05
fake_adam_mink_constants.DEFAULT_STAGE_WARN_THRESHOLD = 0.01
fake_adam_mink_constants.DEFAULT_TIMER_PERIOD = 0.01


class FakeAdamMinkBase:
    def __init__(self, *args, **kwargs) -> None:
        pass

    def _load_config(self, path: str) -> None:
        pass

    def offset_mocap_data(self, data) -> None:
        pass


fake_adam_mink_base.AdamMinkBase = FakeAdamMinkBase
sys.modules.setdefault("adam_mink.adam_mink_base", fake_adam_mink_base)
sys.modules.setdefault("adam_mink.constants", fake_adam_mink_constants)

from primeu_teleop.tracker_retarget_node import (
    CHEST_LINK_NAME,
    LEFT_WRIST_LINK_NAME,
    RELATIVE_CONTROL_LINKS,
    RIGHT_WRIST_LINK_NAME,
    SolveProfile,
    TORSO_RELATIVE_KEY,
    LEFT_ARM_RELATIVE_KEY,
    RIGHT_ARM_RELATIVE_KEY,
    WAIST_LINK_NAME,
    PrimeUTrackerRetarget,
    RelativeControlAnchors,
    TORSO_ACTIVE_JOINTS,
)


class DummyLogger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def warning(self, message: str) -> None:
        self.messages.append(("warning", message))

    def info(self, message: str) -> None:
        self.messages.append(("info", message))


class DummyPrimeUTrackerRetarget:
    _read_ik_entries = staticmethod(PrimeUTrackerRetarget._read_ik_entries)
    _is_placeholder_bone = staticmethod(PrimeUTrackerRetarget._is_placeholder_bone)

    def __init__(self, cfg_path: Path) -> None:
        self._cfg_path = cfg_path
        self._logger = DummyLogger()

    def get_parameter(self, name: str) -> SimpleNamespace:
        assert name == "adam_mink_cfg"
        return SimpleNamespace(value=str(self._cfg_path))

    def get_logger(self) -> DummyLogger:
        return self._logger


class ConfigFilterHarness(PrimeUTrackerRetarget):
    def __init__(self) -> None:
        self._logger = DummyLogger()
        self.adam_mink_cfg = SimpleNamespace(ik_cfg=[])
        self.bone_name_to_cfg = {}
        self._rot_offset_quats = {}
        self._pos_offsets = {}
        self.mocap_data = None

    def get_logger(self) -> DummyLogger:
        return self._logger

    def _initialize_mocap_data(self):
        return {cfg.bone_name: "default" for cfg in self.adam_mink_cfg.ik_cfg}


class RelativeControlHarness(PrimeUTrackerRetarget):
    def __init__(self) -> None:
        self._logger = DummyLogger()
        self.adam_mink_cfg = SimpleNamespace(ik_cfg=[])
        self._relative_control_anchors = None
        self._relative_control_enabled = False
        self._relative_link_to_bone = {}
        self.base_offset_calls = 0
        self._solve_profile = SolveProfile(group_solve_times={})
        self.robot_pose_map = {
            WAIST_LINK_NAME: mink.SE3.from_translation(np.array([0.0, 0.0, 1.0])),
            CHEST_LINK_NAME: mink.SE3.from_translation(np.array([0.0, 0.0, 1.4])),
            LEFT_WRIST_LINK_NAME: mink.SE3.from_translation(np.array([0.4, 0.3, 1.2])),
            RIGHT_WRIST_LINK_NAME: mink.SE3.from_translation(np.array([0.4, -0.3, 1.2])),
        }

    def get_logger(self) -> DummyLogger:
        return self._logger

    def _current_robot_pose(self, link_name: str) -> mink.SE3:
        return self.robot_pose_map[link_name]


def make_cfg(adam_link_name: str, bone_name: str) -> SimpleNamespace:
    return SimpleNamespace(
        adam_link_name=adam_link_name,
        bone_name=bone_name,
        position_cost=1.0,
        orientation_cost=1.0,
    )


def make_pose(position, euler_zyx_deg) -> tuple[np.ndarray, np.ndarray]:
    quat = Rotation.from_euler("zyx", euler_zyx_deg, degrees=True).as_quat(scalar_first=True)
    return np.array(position, dtype=np.float64), quat.astype(np.float64)


def test_get_bone_frames_skips_placeholder_entries(tmp_path: Path) -> None:
    cfg_path = tmp_path / "primeu.yaml"
    cfg_path.write_text(
        """
ik_cfg:
  - adam_link_name: "chest_link"
    bone_name: "TRACKER_TORSO"
  - adam_link_name: "left_wrist_yaw_link"
    bone_name: "LHR-LEFT"
  - adam_link_name: "right_wrist_yaw_link"
    bone_name: "LHR-RIGHT"
""".strip(),
        encoding="utf-8",
    )
    node = DummyPrimeUTrackerRetarget(cfg_path)

    bones = PrimeUTrackerRetarget.get_bone_frames(node)

    assert bones == ["LHR-LEFT", "LHR-RIGHT"]


def test_get_bone_frames_warns_when_all_entries_are_placeholders(tmp_path: Path) -> None:
    cfg_path = tmp_path / "primeu.yaml"
    cfg_path.write_text(
        """
ik_cfg:
  - adam_link_name: "chest_link"
    bone_name: "TRACKER_TORSO"
""".strip(),
        encoding="utf-8",
    )
    node = DummyPrimeUTrackerRetarget(cfg_path)

    bones = PrimeUTrackerRetarget.get_bone_frames(node)

    assert bones == []
    assert any(
        "TRACKER_* placeholders with real tracker IDs" in message
        for level, message in node.get_logger().messages
        if level == "warning"
    )


def test_load_config_filters_placeholder_entries(monkeypatch) -> None:
    harness = ConfigFilterHarness()
    placeholder_cfg = SimpleNamespace(bone_name="TRACKER_TORSO", adam_link_name="chest_link")
    valid_cfg = SimpleNamespace(bone_name="LHR-LEFT", adam_link_name="left_wrist_yaw_link")
    harness.adam_mink_cfg.ik_cfg = [placeholder_cfg, valid_cfg]
    harness.bone_name_to_cfg = {
        placeholder_cfg.bone_name: placeholder_cfg,
        valid_cfg.bone_name: valid_cfg,
    }
    harness._rot_offset_quats = {
        placeholder_cfg.bone_name: "rot_placeholder",
        valid_cfg.bone_name: "rot_valid",
    }
    harness._pos_offsets = {
        placeholder_cfg.bone_name: "pos_placeholder",
        valid_cfg.bone_name: "pos_valid",
    }

    monkeypatch.setattr(
        "primeu_teleop.tracker_retarget_node.AdamMinkBase._load_config",
        lambda self, path: None,
    )

    PrimeUTrackerRetarget._load_config(harness, "/tmp/primeu.yaml")

    assert [cfg.bone_name for cfg in harness.adam_mink_cfg.ik_cfg] == ["LHR-LEFT"]
    assert harness.bone_name_to_cfg == {"LHR-LEFT": valid_cfg}
    assert harness._rot_offset_quats == {"LHR-LEFT": "rot_valid"}
    assert harness._pos_offsets == {"LHR-LEFT": "pos_valid"}
    assert harness.mocap_data == {"LHR-LEFT": "default"}


def test_load_config_rejects_all_placeholder_entries(monkeypatch) -> None:
    harness = ConfigFilterHarness()
    placeholder_cfg = SimpleNamespace(bone_name="TRACKER_TORSO")
    harness.adam_mink_cfg.ik_cfg = [placeholder_cfg]

    monkeypatch.setattr(
        "primeu_teleop.tracker_retarget_node.AdamMinkBase._load_config",
        lambda self, path: None,
    )

    try:
        PrimeUTrackerRetarget._load_config(harness, "/tmp/primeu.yaml")
    except ValueError as exc:
        assert "contains only TRACKER_* placeholders" in str(exc)
    else:
        raise AssertionError("Expected ValueError for all-placeholder config")


def test_configure_relative_control_requires_all_four_trackers() -> None:
    harness = RelativeControlHarness()
    harness.adam_mink_cfg.ik_cfg = [
        make_cfg(WAIST_LINK_NAME, "waist"),
        make_cfg(CHEST_LINK_NAME, "chest"),
        make_cfg(LEFT_WRIST_LINK_NAME, "left"),
    ]

    PrimeUTrackerRetarget._configure_relative_control(harness)

    assert harness._relative_control_enabled is False
    assert harness._anchored_direct_control_enabled is True
    assert any(
        "Anchored direct mode enabled" in message
        for level, message in harness.get_logger().messages
        if level == "info"
    )


def test_configure_relative_control_enables_anchored_direct_mode_for_partial_targets() -> None:
    harness = RelativeControlHarness()
    harness.adam_mink_cfg.ik_cfg = [
        make_cfg(LEFT_WRIST_LINK_NAME, "left"),
        make_cfg(RIGHT_WRIST_LINK_NAME, "right"),
    ]

    PrimeUTrackerRetarget._configure_relative_control(harness)

    assert harness._relative_control_enabled is False
    assert harness._anchored_direct_control_enabled is True
    assert any(
        "Anchored direct mode enabled" in message
        for level, message in harness.get_logger().messages
        if level == "info"
    )


def test_relative_targets_follow_tracker_relative_motion() -> None:
    harness = RelativeControlHarness()
    harness.adam_mink_cfg.ik_cfg = [
        make_cfg(WAIST_LINK_NAME, "waist"),
        make_cfg(CHEST_LINK_NAME, "chest"),
        make_cfg(LEFT_WRIST_LINK_NAME, "left"),
        make_cfg(RIGHT_WRIST_LINK_NAME, "right"),
    ]
    PrimeUTrackerRetarget._configure_relative_control(harness)
    assert harness._relative_control_enabled is True

    harness._relative_control_anchors = RelativeControlAnchors(
        tracker_world={
            WAIST_LINK_NAME: mink.SE3.identity(),
            CHEST_LINK_NAME: mink.SE3.from_translation(np.array([0.0, 0.0, 0.5])),
            LEFT_WRIST_LINK_NAME: mink.SE3.from_translation(np.array([0.3, 0.2, 0.0])),
            RIGHT_WRIST_LINK_NAME: mink.SE3.from_translation(np.array([0.3, -0.2, 0.0])),
        },
        tracker_relatives={
            TORSO_RELATIVE_KEY: mink.SE3.from_translation(np.array([0.0, 0.0, 0.5])),
            LEFT_ARM_RELATIVE_KEY: mink.SE3.from_translation(np.array([0.3, 0.2, -0.5])),
            RIGHT_ARM_RELATIVE_KEY: mink.SE3.from_translation(np.array([0.3, -0.2, -0.5])),
        },
        robot_world={
            name: pose.copy() for name, pose in harness.robot_pose_map.items()
        },
        robot_relatives={
            TORSO_RELATIVE_KEY: harness.robot_pose_map[WAIST_LINK_NAME].inverse()
            @ harness.robot_pose_map[CHEST_LINK_NAME],
            LEFT_ARM_RELATIVE_KEY: harness.robot_pose_map[CHEST_LINK_NAME].inverse()
            @ harness.robot_pose_map[LEFT_WRIST_LINK_NAME],
            RIGHT_ARM_RELATIVE_KEY: harness.robot_pose_map[CHEST_LINK_NAME].inverse()
            @ harness.robot_pose_map[RIGHT_WRIST_LINK_NAME],
        },
    )

    tracker_transforms = {
        WAIST_LINK_NAME: PrimeUTrackerRetarget._se3_from_pose(
            make_pose([0.0, 0.0, 0.0], [30.0, 0.0, 0.0])
        ),
        CHEST_LINK_NAME: PrimeUTrackerRetarget._se3_from_pose(
            make_pose([0.0, 0.0, 0.5], [30.0, 20.0, 0.0])
        ),
        LEFT_WRIST_LINK_NAME: PrimeUTrackerRetarget._se3_from_pose(
            make_pose([0.2, 0.4, 0.2], [40.0, 20.0, 10.0])
        ),
        RIGHT_WRIST_LINK_NAME: PrimeUTrackerRetarget._se3_from_pose(
            make_pose([0.2, -0.4, 0.2], [20.0, 20.0, -10.0])
        ),
    }

    targets = PrimeUTrackerRetarget._build_relative_targets(harness, tracker_transforms)

    waist_yaw = Rotation.from_quat(
        targets[WAIST_LINK_NAME].rotation().parameters(), scalar_first=True
    ).as_euler("zyx", degrees=True)[0]
    left_translation = targets[LEFT_WRIST_LINK_NAME].translation()
    right_translation = targets[RIGHT_WRIST_LINK_NAME].translation()

    assert CHEST_LINK_NAME not in targets
    assert pytest.approx(waist_yaw, abs=1e-6) == 30.0
    assert left_translation[1] > 0.3
    assert right_translation[1] < -0.3


def test_relative_mode_uses_three_ik_tasks_with_chest_as_reference() -> None:
    harness = RelativeControlHarness()
    harness.adam_mink_cfg.ik_cfg = [
        make_cfg(WAIST_LINK_NAME, "waist"),
        make_cfg(CHEST_LINK_NAME, "chest"),
        make_cfg(LEFT_WRIST_LINK_NAME, "left"),
        make_cfg(RIGHT_WRIST_LINK_NAME, "right"),
    ]
    PrimeUTrackerRetarget._configure_relative_control(harness)

    tasks = PrimeUTrackerRetarget._create_ik_tasks(harness)

    assert [task.frame_name for task in tasks] == [
        WAIST_LINK_NAME,
        LEFT_WRIST_LINK_NAME,
        RIGHT_WRIST_LINK_NAME,
    ]
    assert [cfg.adam_link_name for cfg in harness._task_ik_cfg] == [
        WAIST_LINK_NAME,
        LEFT_WRIST_LINK_NAME,
        RIGHT_WRIST_LINK_NAME,
    ]


def test_offset_mocap_data_rewrites_relative_tracker_targets(monkeypatch) -> None:
    harness = RelativeControlHarness()
    harness.adam_mink_cfg.ik_cfg = [
        make_cfg(WAIST_LINK_NAME, "waist"),
        make_cfg(CHEST_LINK_NAME, "chest"),
        make_cfg(LEFT_WRIST_LINK_NAME, "left"),
        make_cfg(RIGHT_WRIST_LINK_NAME, "right"),
    ]
    PrimeUTrackerRetarget._configure_relative_control(harness)

    def fake_base_offset(self, data) -> None:
        self.base_offset_calls += 1

    monkeypatch.setattr(
        "primeu_teleop.tracker_retarget_node.AdamMinkBase.offset_mocap_data",
        fake_base_offset,
    )

    mocap_data = {
        "waist": make_pose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        "chest": make_pose([0.0, 0.0, 0.5], [0.0, 0.0, 0.0]),
        "left": make_pose([0.3, 0.2, 0.0], [0.0, 0.0, 0.0]),
        "right": make_pose([0.3, -0.2, 0.0], [0.0, 0.0, 0.0]),
    }

    PrimeUTrackerRetarget.offset_mocap_data(harness, mocap_data)

    assert harness.base_offset_calls == 1
    assert harness._relative_control_anchors is not None

    updated_waist_pos, updated_waist_quat = mocap_data["waist"]
    updated_chest_pos, _ = mocap_data["chest"]
    updated_left_pos, _ = mocap_data["left"]
    assert np.allclose(updated_waist_pos, np.array([0.0, 0.0, 1.0]))
    assert np.allclose(updated_chest_pos, np.array([0.0, 0.0, 0.5]))
    assert np.allclose(updated_left_pos, np.array([0.4, 0.3, 1.2]))
    assert np.allclose(updated_waist_quat, np.array([1.0, 0.0, 0.0, 0.0]))


def test_offset_mocap_data_rewrites_anchored_direct_targets(monkeypatch) -> None:
    harness = RelativeControlHarness()
    harness.adam_mink_cfg.ik_cfg = [
        make_cfg(LEFT_WRIST_LINK_NAME, "left"),
        make_cfg(RIGHT_WRIST_LINK_NAME, "right"),
    ]
    PrimeUTrackerRetarget._configure_relative_control(harness)
    assert harness._anchored_direct_control_enabled is True

    def fake_base_offset(self, data) -> None:
        self.base_offset_calls += 1

    monkeypatch.setattr(
        "primeu_teleop.tracker_retarget_node.AdamMinkBase.offset_mocap_data",
        fake_base_offset,
    )

    mocap_data = {
        "left": make_pose([1.0, 2.0, 3.0], [0.0, 0.0, 0.0]),
        "right": make_pose([-1.0, -2.0, 3.0], [0.0, 0.0, 0.0]),
    }
    PrimeUTrackerRetarget.offset_mocap_data(harness, mocap_data)
    assert harness.base_offset_calls == 1
    assert harness._direct_control_anchors is not None

    mocap_data = {
        "left": make_pose([1.1, 2.2, 2.9], [15.0, 0.0, 0.0]),
        "right": make_pose([-1.0, -2.1, 3.0], [0.0, 0.0, 0.0]),
    }
    PrimeUTrackerRetarget.offset_mocap_data(harness, mocap_data)

    updated_left_pos, updated_left_quat = mocap_data["left"]
    updated_right_pos, _ = mocap_data["right"]
    left_yaw = Rotation.from_quat(updated_left_quat, scalar_first=True).as_euler(
        "zyx", degrees=True
    )[0]

    assert np.allclose(updated_left_pos, np.array([0.5, 0.5, 1.1]))
    assert np.allclose(updated_right_pos, np.array([0.4, -0.4, 1.2]))
    assert pytest.approx(left_yaw, abs=1e-6) == 15.0


def test_configured_active_joint_names_freezes_torso_without_torso_target() -> None:
    harness = RelativeControlHarness()
    harness.adam_mink_cfg.ik_cfg = [
        make_cfg(LEFT_WRIST_LINK_NAME, "left"),
        make_cfg(RIGHT_WRIST_LINK_NAME, "right"),
    ]

    active_joint_names = PrimeUTrackerRetarget._configured_active_joint_names(harness)

    assert not (TORSO_ACTIVE_JOINTS & active_joint_names)


def test_configured_active_joint_names_enables_torso_with_waist_target() -> None:
    harness = RelativeControlHarness()
    harness.adam_mink_cfg.ik_cfg = [
        make_cfg(LEFT_WRIST_LINK_NAME, "left"),
        make_cfg(RIGHT_WRIST_LINK_NAME, "right"),
        make_cfg(WAIST_LINK_NAME, "waist"),
    ]

    active_joint_names = PrimeUTrackerRetarget._configured_active_joint_names(harness)

    assert TORSO_ACTIVE_JOINTS.issubset(active_joint_names)
