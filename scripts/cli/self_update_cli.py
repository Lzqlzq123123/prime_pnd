# scripts/cli/self_update_cli.py — pteleop self-update command

from __future__ import annotations

import os
import platform
import shutil
import subprocess
import tarfile
import tempfile
from pathlib import Path

import typer

from .common import PROJECT_ROOT, echo_error, echo_info, echo_success, get_version

RELEASE_BASE_URL = os.environ.get(
    "PND_RELEASE_URL",
    "https://github.com/pndbotics/pnd_teleoperation/releases/latest/download",
)


def _current_version() -> str | None:
    v = get_version()
    return v if v != "unknown" else None


def _detect_arch() -> str:
    machine = platform.machine()
    return {"x86_64": "amd64", "aarch64": "arm64"}.get(machine, machine)


def _fetch_latest_version() -> str | None:
    """Resolve the GitHub /releases/latest redirect to get the remote version tag."""
    base = RELEASE_BASE_URL
    latest_url = base.rsplit("/download", 1)[0] if base.endswith("/download") else None
    if not latest_url:
        return None
    if shutil.which("curl"):
        r = subprocess.run(
            ["curl", "-sIL", "-o", "/dev/null", "-w", "%{url_effective}", latest_url],
            capture_output=True,
            text=True,
            check=False,
            timeout=10,
        )
        if r.returncode == 0:
            tag = r.stdout.strip().rsplit("/", 1)[-1]
            return tag.lstrip("v") if tag else None
    return None


def _download(url: str, dest: Path) -> None:
    """Download a file using curl or wget."""
    if shutil.which("curl"):
        r = subprocess.run(
            ["curl", "-fSL", "--progress-bar", "-o", str(dest), url],
            check=False,
        )
        if r.returncode != 0:
            echo_error(f"Download failed: {url}")
            raise typer.Exit(1)
    elif shutil.which("wget"):
        r = subprocess.run(
            ["wget", "-q", "--show-progress", "-O", str(dest), url],
            check=False,
        )
        if r.returncode != 0:
            echo_error(f"Download failed: {url}")
            raise typer.Exit(1)
    else:
        echo_error("curl or wget is required")
        raise typer.Exit(1)


def _extract_tarball(tarball: Path, dest: Path) -> None:
    with tarfile.open(tarball, "r:gz") as tf:
        members = tf.getnames()
        prefix = os.path.commonpath(members) if members else ""
        if prefix and prefix != ".":
            for member in tf.getmembers():
                member.path = os.path.relpath(member.path, prefix)
                if member.linkname:
                    member.linkname = os.path.relpath(member.linkname, prefix)
                tf.extract(member, dest, filter="data")
        else:
            tf.extractall(dest, filter="data")


def _run_uv_sync() -> None:
    if not shutil.which("uv"):
        echo_error("uv not found. Install: curl -LsSf https://astral.sh/uv/install.sh | sh")
        raise typer.Exit(1)
    echo_info("Running uv sync ...")
    r = subprocess.run(["uv", "sync", "--python", "3.10", "--quiet"], cwd=str(PROJECT_ROOT))
    if r.returncode != 0:
        echo_error("uv sync failed")
        raise typer.Exit(1)


def self_update(
    file: Path | None = typer.Option(
        None,
        "--file",
        "-f",
        help="Upgrade from a local tarball instead of downloading.",
        exists=True,
        readable=True,
    ),
    yes: bool = typer.Option(False, "--yes", "-y", help="Skip confirmation prompt."),
) -> None:
    """Update pnd-teleop to the latest version.

    \b
    Examples:
      spteleop self-update                       # download latest
      spteleop self-update --file release.tar.gz  # from local file
    """
    old_version = _current_version() or "unknown"
    print(f"old_version: {old_version}")
    arch = _detect_arch()

    if not file:
        remote_version = _fetch_latest_version()
        print(f"remote_version: {remote_version}")
        if remote_version and old_version != "unknown" and remote_version == old_version:
            echo_success(f"Already up to date (v{old_version})")
            return

    if file:
        echo_info(f"Upgrading from local file: {file.name}")
        tarball_path = file
        cleanup = False
    else:
        url = f"{RELEASE_BASE_URL}/pnd-teleop-{arch}.tar.gz"
        echo_info(f"Downloading latest release ({arch}) ...")
        tarball_path = Path(tempfile.mktemp(suffix=".tar.gz", prefix="pnd-teleop-update-"))
        cleanup = True
        _download(url, tarball_path)

    if not yes:
        typer.confirm(
            f"Upgrade pnd-teleop v{old_version} at {PROJECT_ROOT}?",
            default=True,
            abort=True,
        )

    echo_info("Extracting ...")
    _extract_tarball(tarball_path, PROJECT_ROOT)

    if cleanup:
        tarball_path.unlink(missing_ok=True)

    _run_uv_sync()

    new_version = _current_version() or "unknown"
    if old_version == new_version:
        echo_success(f"Already up to date (v{new_version})")
    else:
        echo_success(f"Updated: v{old_version} → v{new_version}")


def register(app: typer.Typer) -> None:
    app.command("self-update")(self_update)
