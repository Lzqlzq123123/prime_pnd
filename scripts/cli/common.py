# scripts/cli/common.py — project root, output style, launch runner

import os
import subprocess
import sys
from pathlib import Path

import typer

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


# ---------- Version ----------
def get_version() -> str:
    from importlib.metadata import PackageNotFoundError, version

    try:
        return version("pnd-teleop")
    except PackageNotFoundError:
        pass
    for path in (PROJECT_ROOT / ".precompiled", PROJECT_ROOT / ".version"):
        if path.is_file():
            v = path.read_text().strip()
            if v:
                return v
    return "unknown"


# ---------- Output styling ----------
def echo_info(msg: str) -> None:
    typer.echo(typer.style("[INFO] ", fg=typer.colors.BLUE) + msg)


def echo_success(msg: str) -> None:
    typer.echo(typer.style("[SUCCESS] ", fg=typer.colors.GREEN) + msg)


def echo_error(msg: str) -> None:
    typer.echo(typer.style("[ERROR] ", fg=typer.colors.RED) + msg)


def echo_warning(msg: str) -> None:
    typer.echo(typer.style("[WARNING] ", fg=typer.colors.YELLOW) + msg)


# ---------- Conda / environment helpers ----------
def detect_conda() -> bool:
    """Return True if a conda environment is currently active."""
    return bool(os.environ.get("CONDA_DEFAULT_ENV") or os.environ.get("CONDA_PREFIX"))


def _deactivate_conda_shell() -> str:
    """Return shell snippet that fully deactivates conda before sourcing ROS.

    Conda's Python (e.g. 3.11) conflicts with ROS Humble's Python 3.10
    C extensions (rclpy pybind11). We deactivate all nested conda envs
    and scrub conda paths from PATH so the system Python 3.10 is used.
    """
    return (
        'if [ -n "${CONDA_EXE:-}" ] && type conda &>/dev/null; then '
        "while [ -n \"${CONDA_DEFAULT_ENV:-}\" ] && [ \"${CONDA_DEFAULT_ENV}\" != \"\" ]; do "
        "conda deactivate 2>/dev/null || break; "
        "done; "
        "fi; "
        'export PATH=$(echo "$PATH" | tr ":" "\\n" | grep -v -E "(conda|anaconda|miniconda|miniforge)" | tr "\\n" ":"); '
        'export PATH="${PATH%:}"; '
        'unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE; '
    )


# ---------- ROS launch execution ----------
ROS_PYTHON_VERSION = "python3.10"


def _venv_site_packages_cmd() -> str:
    """Return shell snippet that adds the .venv site-packages to PYTHONPATH.

    We must NOT run ``source .venv/bin/activate`` because it prepends
    .venv/bin to PATH, overriding the system python3 with whatever
    interpreter the venv was created with (possibly conda Python 3.11/3.12).
    ROS Humble nodes require the system Python 3.10, so we only expose
    the venv's Python 3.10 site-packages via PYTHONPATH.  Adding packages
    built for a different Python ABI (e.g. numpy cpython-312) would crash.
    """
    venv_sp = PROJECT_ROOT / ".venv" / "lib"
    if not venv_sp.is_dir():
        return "true"
    sp_dir = venv_sp / ROS_PYTHON_VERSION / "site-packages"
    return (
        f'if [ -d "{sp_dir}" ]; then '
        f'export PYTHONPATH="{sp_dir}:${{PYTHONPATH:-}}"; '
        "fi; "
    )


def run_launch_shell(launch_cmd: str) -> int:
    """Run a launch command in bash after sourcing ROS and workspace."""
    deactivate_conda = _deactivate_conda_shell() if detect_conda() else ""
    add_venv_packages = _venv_site_packages_cmd()
    shell_cmd = (
        "set -e; "
        f"{deactivate_conda}"
        "source /opt/ros/humble/setup.bash; "
        f"source {PROJECT_ROOT}/install/setup.bash; "
        "export ROS_LOCALHOST_ONLY=1; "
        f"{add_venv_packages}"
        f"exec {launch_cmd}"
    )
    return subprocess.run(
        ["bash", "-c", shell_cmd],
        cwd=str(PROJECT_ROOT),
    ).returncode


def echo_launch_and_exit(launch_cmd: str) -> None:
    """Print launch info and exit with the launch process return code."""
    echo_success("Environment ready")
    echo_info(f"Run: {launch_cmd}")
    echo_info("Press Ctrl+C to stop...")
    typer.echo()
    sys.exit(run_launch_shell(launch_cmd))
