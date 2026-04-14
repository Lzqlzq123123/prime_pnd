# scripts/cli/caddy.py — background Caddy with cleanup

import atexit
import contextlib
import os
import shutil
import signal
import subprocess
import sys
import time

import typer

from .common import PROJECT_ROOT, echo_error, echo_info, echo_success

# PID of Caddy started by this process (for cleanup on exit)
_caddy_pid: int | None = None


def _check_caddy() -> bool:
    return shutil.which("caddy") is not None


def _find_running_caddy_pids() -> list[int]:
    """Return PIDs of running caddy processes on the system."""
    # Prefer pgrep (faster), else fall back to ps.
    try:
        p = subprocess.run(
            ["pgrep", "-x", "caddy"],
            capture_output=True,
            text=True,
            timeout=1,
        )
        if p.returncode == 0 and p.stdout.strip():
            return [int(x) for x in p.stdout.split() if x.strip().isdigit()]
        return []
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    try:
        p = subprocess.run(
            ["ps", "-C", "caddy", "-o", "pid="],
            capture_output=True,
            text=True,
            timeout=1,
        )
        if p.returncode == 0 and p.stdout.strip():
            return [int(x) for x in p.stdout.split() if x.strip().isdigit()]
        return []
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []


def _kill_running_caddy(pids: list[int]) -> None:
    """Terminate running caddy processes.

    Try SIGTERM without sudo first; use sudo kill if that fails.
    """
    if not pids:
        return

    echo_info(f"Stopping existing Caddy (PID(s): {', '.join(map(str, pids))}) ...")

    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            with contextlib.suppress(FileNotFoundError, subprocess.TimeoutExpired):
                subprocess.run(
                    ["sudo", "kill", "-TERM", str(pid)],
                    capture_output=True,
                    timeout=2,
                )
        except Exception:
            pass

    # Wait up to 2s for processes to exit
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if not _find_running_caddy_pids():
            return
        time.sleep(0.1)

    # If still running, escalate to SIGKILL
    remaining = _find_running_caddy_pids()
    if remaining:
        echo_info(
            f"Caddy still running, force killing PID(s): {', '.join(map(str, remaining))}) ..."
        )
        for pid in remaining:
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                with contextlib.suppress(FileNotFoundError, subprocess.TimeoutExpired):
                    subprocess.run(
                        ["sudo", "kill", "-KILL", str(pid)],
                        capture_output=True,
                        timeout=2,
                    )
            except Exception:
                pass


def _kill_caddy() -> None:
    """On exit, terminate Caddy started by this process (including its process group)."""
    global _caddy_pid
    if _caddy_pid is None:
        return
    pid = _caddy_pid
    _caddy_pid = None
    try:
        # Kill process group (Caddy may be started by start_caddy.sh in a child)
        os.killpg(pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        with contextlib.suppress(FileNotFoundError, subprocess.TimeoutExpired):
            subprocess.run(
                ["sudo", "kill", "-TERM", f"-{pid}"],
                capture_output=True,
                timeout=2,
            )
    except Exception:
        pass


def _install_caddy_cleanup_on_signal() -> None:
    """On SIGINT/SIGTERM, exit; atexit cleans up Caddy."""

    def _handler(signum: int, frame: object) -> None:
        sys.exit(128 + (signum if signum < 128 else 0))

    with contextlib.suppress(ValueError, OSError):
        signal.signal(signal.SIGINT, _handler)
    with contextlib.suppress(ValueError, OSError):
        signal.signal(signal.SIGTERM, _handler)


def _start_caddy_background() -> None:
    if not _check_caddy():
        echo_error("Caddy is not installed or not in PATH")
        typer.echo("Install docs: https://caddyserver.com/docs/install#debian-ubuntu-raspbian")
        raise typer.Exit(1)

    # If caddy is already running, kill it first then restart to avoid port conflicts / config mismatch.
    running_pids = _find_running_caddy_pids()
    if running_pids:
        _kill_running_caddy(running_pids)

    echo_info("Starting Caddy in background...")
    global _caddy_pid
    caddy_script = PROJECT_ROOT / "scripts" / "start_caddy.sh"
    if not caddy_script.is_file():
        echo_error(f"Caddy script not found: {caddy_script}")
        raise typer.Exit(1)
    try:
        proc = subprocess.Popen(
            ["sudo", "bash", str(caddy_script)],
            cwd=str(PROJECT_ROOT),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        _caddy_pid = proc.pid
        atexit.register(_kill_caddy)
        _install_caddy_cleanup_on_signal()
        echo_success(f"Caddy started in background (PID: {proc.pid})")
    except Exception as e:
        echo_error(f"Failed to start Caddy: {e}")
        raise typer.Exit(1) from None

    # Briefly verify the process actually started (avoid silent failure if the script exits immediately).
    for _ in range(10):
        if proc.poll() is not None:
            echo_error(
                "Caddy exited immediately after start. Check sudo permission or port conflicts."
            )
            raise typer.Exit(1)
        if _find_running_caddy_pids():
            return
        try:
            time.sleep(0.1)
        except Exception:
            break
