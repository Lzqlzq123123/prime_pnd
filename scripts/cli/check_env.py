# scripts/cli/check_env.py — ROS environment checks

from pathlib import Path

import typer

from .common import PROJECT_ROOT, detect_conda, echo_error, echo_info, echo_warning


def _is_precompiled() -> bool:
    return (PROJECT_ROOT / ".precompiled").is_file()


def _check_ros_environment() -> None:
    ros_setup = Path("/opt/ros/humble/setup.bash")
    if not ros_setup.is_file():
        echo_error("ROS Humble not found: /opt/ros/humble/setup.bash")
        raise typer.Exit(1)
    install_setup = PROJECT_ROOT / "install" / "setup.bash"
    if not install_setup.is_file():
        if _is_precompiled():
            echo_error(
                "Missing install/setup.bash. The pre-compiled release may be incomplete. "
                "Re-install or run: spteleop self-update"
            )
        else:
            echo_error(
                "Missing install/setup.bash. Please build the workspace first. run ./build.sh"
            )
        raise typer.Exit(1)
    venv_activate = PROJECT_ROOT / ".venv" / "bin" / "activate"
    if not venv_activate.is_file():
        if _is_precompiled():
            echo_error("Missing .venv/bin/activate. Run: spteleop self-update")
        else:
            echo_error("Missing .venv/bin/activate. Please run uv sync")
        raise typer.Exit(1)
    if detect_conda():
        echo_warning(
            "Conda environment detected. Conda will be auto-deactivated before launching ROS nodes "
            "to avoid Python version conflicts (ROS Humble requires Python 3.10)."
        )
    if _is_precompiled():
        echo_info("Running from pre-compiled release")
