"""Launch the PrimeU tracker teleop pipeline.

This brings up:
  - primeu_teleop tracker_retarget_node (Mink IK)
  - (optional) MuJoCo passive viewer (set mujoco_sim:=true)
  - (optional) RViz with a default config

Assumes libsurvive_ros2 is started separately, e.g.:
    ros2 launch libsurvive_ros2 libsurvive_ros2.launch.py
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("primeu_teleop"))

    # Try to find primeu_description, fallback to source if not built
    try:
        primeu_desc_share = Path(get_package_share_directory("primeu_description"))
        #定义了机器人模型的路径
        default_model = primeu_desc_share / "mjcf" / "primeu_robot.xml"
        default_urdf = primeu_desc_share / "urdf" / "primeu_robot.urdf"
    except:
        # Fallback to source directory
        default_model = Path("/home/lzq/pnd_teleoperation/src/visualization/primeu_description/mjcf/primeu_robot.xml")
        default_urdf = Path("/home/lzq/pnd_teleoperation/src/visualization/primeu_description/urdf/primeu_robot.urdf")
    
    # Mink IK 算法的默认配置文件路径
    default_cfg = pkg_share / "config" / "primeu_minimal.yaml"

    # Find the virtualenv Python site-packages
    venv_path = Path("/home/lzq/pnd_teleoperation/.venv")
    python_version = f"python{os.sys.version_info.major}.{os.sys.version_info.minor}"
    venv_site_packages = venv_path / "lib" / python_version / "site-packages"

    # Prepend venv site-packages to PYTHONPATH so mink/mujoco are found
    current_pythonpath = os.environ.get("PYTHONPATH", "")
    new_pythonpath = f"{venv_site_packages}:{current_pythonpath}" if current_pythonpath else str(venv_site_packages)

    default_root_link = "body_base_link"

    # 启动参数
    return LaunchDescription([
        # Set PYTHONPATH to include virtualenv packages
        SetEnvironmentVariable("PYTHONPATH", new_pythonpath),

        DeclareLaunchArgument(
            "robot_model",
            default_value=str(default_model),
            description="Path to PrimeU MuJoCo XML model.",
        ),
        DeclareLaunchArgument(
            "ik_config",
            default_value=str(default_cfg),
            description="Path to Mink IK YAML config.",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="libsurvive_world",
            description="Common world frame used for tracker targets and robot visualization.",
        ),
        DeclareLaunchArgument(
            "root_link",
            default_value=default_root_link,
            description="Robot root link used by adam_state_publisher.",
        ),
        DeclareLaunchArgument(
            "mujoco_sim",
            default_value="false",
            description="Start the MuJoCo passive viewer.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "ik_solver",
            default_value="daqp",
            description="QP solver: daqp / osqp / proxqp / quadprog.",
        ),
        DeclareLaunchArgument(
            "ik_iter_max",
            default_value="1",
            description="Maximum IK iterations per cycle.",
        ),
        DeclareLaunchArgument(
            "ik_damping",
            default_value="0.3",
            description="IK damping factor.",
        ),
        DeclareLaunchArgument(
            "split_upper_body_ik",
            default_value="false",
            description="Deprecated and ignored. Unified multi-task IK is always used.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_velocity_limits",
            default_value="true",
            description="Enable Mink VelocityLimit constraints.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_configuration_limits",
            default_value="true",
            description="Enable Mink ConfigurationLimit constraints.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "anchor_targets_to_initial_pose",    
            default_value="true",
            description="Anchor tracker targets to the robot pose captured on the first frame.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "publish_tracker_relative_pose",
            default_value="false",
            description="Publish per-tracker anchor-relative PoseStamped diagnostics.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "tracker_translation_scale",
            default_value="0.6",
            description="Scale applied to tracker translation deltas before retargeting.",
        ),
        DeclareLaunchArgument(
            "ik_loop_hz",
            default_value="100.0",
            description="Target IK loop frequency in Hz.",
        ),
        DeclareLaunchArgument(
            "ik_dt",
            default_value="0.0",
            description="IK integration dt in seconds. Use 0.0 to follow ik_loop_hz.",
        ),
        DeclareLaunchArgument(
            "qp_iter_limit",
            default_value="80",
            description="DAQP maximum iterations per IK solve. Use 0 for solver default.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz alongside the teleop node.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "world_to_libsurvive_x",
            default_value="0",
            description="Static translation x from world to libsurvive_world.",
        ),
        DeclareLaunchArgument(
            "world_to_libsurvive_y",
            default_value="0",
            description="Static translation y from world to libsurvive_world.",
        ),
        DeclareLaunchArgument(
            "world_to_libsurvive_z",
            default_value="0",
            description="Static translation z from world to libsurvive_world.",
        ),
        DeclareLaunchArgument(
            "world_to_libsurvive_qx",
            default_value="0",
            description="Static quaternion x from world to libsurvive_world.",
        ),
        DeclareLaunchArgument(
            "world_to_libsurvive_qy",
            default_value="0",
            description="Static quaternion y from world to libsurvive_world.",
        ),
        DeclareLaunchArgument(
            "world_to_libsurvive_qz",
            default_value="0",
            description="Static quaternion z from world to libsurvive_world.",
        ),
        DeclareLaunchArgument(
            "world_to_libsurvive_qw",
            default_value="1",
            description="Static quaternion w from world to libsurvive_world.",
        ),

        # Main retarget node 启动了 tracker_retarget 节点
        Node(
            package="primeu_teleop",
            executable="tracker_retarget",
            name="primeu_tracker_retarget",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "adam_model_path": ParameterValue(
                        LaunchConfiguration("robot_model"),
                        value_type=str,
                    ),
                    "adam_mink_cfg": ParameterValue(
                        LaunchConfiguration("ik_config"),
                        value_type=str,
                    ),
                    "base_frame": ParameterValue(
                        LaunchConfiguration("base_frame"),
                        value_type=str,
                    ),
                    "mujoco_sim": ParameterValue(
                        LaunchConfiguration("mujoco_sim"),
                        value_type=bool,
                    ),
                    "ik_solver": ParameterValue(
                        LaunchConfiguration("ik_solver"),
                        value_type=str,
                    ),
                    "ik_iter_max": ParameterValue(
                        LaunchConfiguration("ik_iter_max"),
                        value_type=int,
                    ),
                    "ik_damping": ParameterValue(
                        LaunchConfiguration("ik_damping"),
                        value_type=float,
                    ),
                    "split_upper_body_ik": ParameterValue(
                        LaunchConfiguration("split_upper_body_ik"),
                        value_type=bool,
                    ),
                    "use_velocity_limits": ParameterValue(
                        LaunchConfiguration("use_velocity_limits"),
                        value_type=bool,
                    ),
                    "use_configuration_limits": ParameterValue(
                        LaunchConfiguration("use_configuration_limits"),
                        value_type=bool,
                    ),
                    "anchor_targets_to_initial_pose": ParameterValue(
                        LaunchConfiguration("anchor_targets_to_initial_pose"),
                        value_type=bool,
                    ),
                    "publish_tracker_relative_pose": ParameterValue(
                        LaunchConfiguration("publish_tracker_relative_pose"),
                        value_type=bool,
                    ),
                    "tracker_translation_scale": ParameterValue(
                        LaunchConfiguration("tracker_translation_scale"),
                        value_type=float,
                    ),
                    "ik_loop_hz": ParameterValue(
                        LaunchConfiguration("ik_loop_hz"),
                        value_type=float,
                    ),
                    "ik_dt": ParameterValue(
                        LaunchConfiguration("ik_dt"),
                        value_type=float,
                    ),
                    "qp_iter_limit": ParameterValue(
                        LaunchConfiguration("qp_iter_limit"),
                        value_type=int,
                    ),
                }
            ],
        ),

        # Align the Lighthouse tracking world with the shared robot world frame.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_libsurvive_world",
            output="screen",
            condition=IfCondition(
                PythonExpression(
                    ["'", LaunchConfiguration("base_frame"), "' != 'libsurvive_world'"]
                )
            ),
            arguments=[
                LaunchConfiguration("world_to_libsurvive_x"),
                LaunchConfiguration("world_to_libsurvive_y"),
                LaunchConfiguration("world_to_libsurvive_z"),
                LaunchConfiguration("world_to_libsurvive_qx"),
                LaunchConfiguration("world_to_libsurvive_qy"),
                LaunchConfiguration("world_to_libsurvive_qz"),
                LaunchConfiguration("world_to_libsurvive_qw"),
                LaunchConfiguration("base_frame"),
                "libsurvive_world",
            ],
        ),

        # Adam state publisher - understands root_pos/root_quat joint-state fields.
        Node(
            package="adam_state_publisher",
            executable="adam_state_publisher",
            name="adam_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": open(
                        str(default_urdf)
                    ).read(),
                    "root_link": LaunchConfiguration("root_link"),
                }
            ],
        ),

        # RViz (optional)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])




