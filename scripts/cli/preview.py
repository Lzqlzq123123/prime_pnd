import atexit
import contextlib
import os
import signal
import socket
import subprocess
import sys
import time

import typer

from .common import (
    PROJECT_ROOT,
    _deactivate_conda_shell,
    _venv_site_packages_cmd,
    detect_conda,
    echo_error,
    echo_info,
    echo_success,
)

_preview_pid: int | None = None


def _kill_preview() -> None:
    """Terminate foxglove_bridge (and its process group) started by this process."""
    global _preview_pid
    if _preview_pid is None:
        return
    pid = _preview_pid
    _preview_pid = None
    try:
        os.killpg(pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        with contextlib.suppress(FileNotFoundError, subprocess.TimeoutExpired):
            subprocess.run(
                ["sudo", "kill", "-TERM", f"-{pid}"],
                capture_output=True,
                timeout=2,
            )
    except Exception:
        # Avoid cleanup failures during exit affecting main logic
        pass


def _install_preview_cleanup_on_signal() -> None:
    """On SIGINT/SIGTERM, exit; atexit cleans up foxglove_bridge."""

    def _handler(signum: int, frame: object) -> None:  # type: ignore[unused-argument]
        sys.exit(128 + (signum if signum < 128 else 0))

    with contextlib.suppress(ValueError, OSError):
        signal.signal(signal.SIGINT, _handler)
    with contextlib.suppress(ValueError, OSError):
        signal.signal(signal.SIGTERM, _handler)


def _start_preview_background() -> None:
    """Start foxglove_bridge in the background for Foxglove Studio preview."""
    global _preview_pid
    if _preview_pid is not None:
        echo_info("Foxglove preview is already running in the background")
        return

    echo_info("Starting Foxglove preview in background (foxglove_bridge)...")

    deactivate_conda = _deactivate_conda_shell() if detect_conda() else ""
    add_venv_packages = _venv_site_packages_cmd()
    shell_cmd = (
        "set -e; "
        f"{deactivate_conda}"
        "source /opt/ros/humble/setup.bash; "
        f"source {PROJECT_ROOT}/install/setup.bash; "
        "export ROS_LOCALHOST_ONLY=1; "
        f"{add_venv_packages}"
        "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
    )

    try:
        proc = subprocess.Popen(
            ["bash", "-c", shell_cmd],
            cwd=str(PROJECT_ROOT),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        _preview_pid = proc.pid
        atexit.register(_kill_preview)
        _install_preview_cleanup_on_signal()
        echo_success(f"Foxglove preview started in background (PID: {proc.pid})")
    except Exception as e:  # pragma: no cover - defensive error path
        echo_error(f"Failed to start Foxglove preview: {e}")
        raise typer.Exit(1) from None

    # Verify foxglove_bridge actually started: process still running and port accepts connections
    for _ in range(30):  # ~3s
        if proc.poll() is not None:
            echo_error(
                "foxglove_bridge exited immediately after start. Check ROS environment and dependencies."
            )
            raise typer.Exit(1)
        try:
            with socket.create_connection(("127.0.0.1", 8765), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)

    echo_error(
        "foxglove_bridge did not listen on 127.0.0.1:8765 in time. Check port conflicts or startup logs."
    )
    raise typer.Exit(1)
