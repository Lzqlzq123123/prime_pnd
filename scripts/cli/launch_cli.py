# scripts/cli/launch_cli.py — launch command

from __future__ import annotations

import typer

from .caddy import _start_caddy_background
from .check_env import _check_ros_environment
from .common import echo_error, echo_info, echo_launch_and_exit, echo_warning
from .ensure_meshes import adam_type_from_launch_stem, ensure_meshes
from .preview import _start_preview_background
from .teleop_cmd import (
    _complete_launch_stem,
    _get_all_launch_stems,
    _get_launch_command_by_stem,
)


def launch(
    launch_stem: str = typer.Argument(
        ...,
        help="Launch file stem, e.g. pinocchio-adam_u-noitom, mink-adam_u-webvr. Tab-completable.",
        autocompletion=_complete_launch_stem,
    ),
    with_preview: bool = typer.Option(
        True,
        "--with-preview/--no-preview",
        help="Whether to start Foxglove preview (foxglove_bridge) in background.",
    ),
) -> None:
    """Select and run a bringup launch file by stem (Tab lists all bringup launch files).

    pteleop launch pinocchio-adam_u-noitom
    pteleop launch test_retarget_vr
    """
    all_stems = _get_all_launch_stems()
    if not all_stems:
        echo_error(
            "No bringup launch files found. Build the workspace (colcon build) or check src/bringup/launch."
        )
        raise typer.Exit(1)

    launch_cmd = _get_launch_command_by_stem(launch_stem)
    if not launch_cmd:
        echo_error(f"Unknown launch: {launch_stem}")
        echo_info("Bringup launch files (stem):")
        for s in all_stems:
            typer.echo(f"  {s}")
        raise typer.Exit(1)

    echo_info("Checking ROS environment...")
    _check_ros_environment()

    adam_type = adam_type_from_launch_stem(launch_stem)
    if adam_type and not ensure_meshes(adam_type):
        echo_warning("Mesh download failed; launch may fail if meshes are missing.")

    if with_preview:
        _start_preview_background()
    if "mink-adam_u-webvr" in launch_stem:
        _start_caddy_background()

    echo_launch_and_exit(launch_cmd)


def register(app: typer.Typer) -> None:
    app.command()(launch)
