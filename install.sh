#!/usr/bin/env bash
# PND Teleop — Quick Start Installer
#
# Install pre-compiled pnd_teleoperation without building from source.
#
# Usage:
#   curl -fsSL https://pndbotics.com/install.sh | bash
#   ./quickstart.sh
#   ./quickstart.sh --file pnd-teleop-amd64.tar.gz
#   ./quickstart.sh --install-dir ~/my_teleop
#
# Prerequisites: Ubuntu 22.04, ROS 2 Humble, Python 3.10

set -euo pipefail

INSTALLER_VERSION="0.1.0"
DEFAULT_INSTALL_DIR="/opt/pnd/pnd_teleop"
RELEASE_BASE_URL="${PND_RELEASE_URL:-https://github.com/pndbotics/pnd_teleoperation/releases/latest/download}"

# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'
BOLD='\033[1m'; DIM='\033[2m'; NC='\033[0m'
info()   { echo -e "${BLUE}info${NC}: $1"; }
warn()   { echo -e "${YELLOW}warn${NC}: $1" >&2; }
err()    { echo -e "${RED}error${NC}: $1" >&2; }
fail()   { err "$1"; exit 1; }

# ---------------------------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------------------------
LOCAL_FILE=""
INSTALL_DIR=""

usage() {
  cat <<EOF
pnd-teleop-installer $INSTALLER_VERSION

Install pre-compiled pnd_teleoperation without building from source.

USAGE:
    quickstart.sh [OPTIONS]

OPTIONS:
    --file <PATH>          Install from a local tarball instead of downloading
    --install-dir <PATH>   Installation directory [default: /opt/pnd/pnd_teleop]
    -h, --help             Show this help message
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --file)        LOCAL_FILE="$2";   shift 2 ;;
    --install-dir) INSTALL_DIR="$2";  shift 2 ;;
    -h|--help)     usage; exit 0 ;;
    *) fail "unknown option: $1. Use --help for usage." ;;
  esac
done

INSTALL_DIR="${INSTALL_DIR:-${DEFAULT_INSTALL_DIR}}"

# ---------------------------------------------------------------------------
# Detect platform
# ---------------------------------------------------------------------------
detect_arch() {
  local arch
  arch=$(uname -m)
  case "$arch" in
    x86_64)  echo "amd64" ;;
    aarch64) echo "arm64" ;;
    *)       echo "$arch" ;;
  esac
}

ARCH=$(detect_arch)

# ---------------------------------------------------------------------------
# Prerequisites
# ---------------------------------------------------------------------------
echo ""
echo -e "${BOLD}pnd-teleop installer${NC} ${DIM}${INSTALLER_VERSION}${NC}"
echo ""

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  fail "ROS 2 Humble not found.
  Install it first:
    sudo apt update && sudo apt install ros-humble-desktop"
fi

if ! command -v python3 &>/dev/null; then
  fail "python3 not found. Install: sudo apt install python3 python3-pip"
fi

# ---------------------------------------------------------------------------
# Conda guard
# ---------------------------------------------------------------------------
if [[ -n "${CONDA_DEFAULT_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
  warn "conda environment detected — deactivating (ROS Humble requires system Python 3.10)"
  if type conda &>/dev/null; then
    while [[ -n "${CONDA_DEFAULT_ENV:-}" ]]; do
      conda deactivate 2>/dev/null || break
    done
  fi
  export PATH=$(echo "$PATH" | tr ":" "\n" | grep -v -E "(conda|anaconda|miniconda|miniforge)" | tr "\n" ":")
  export PATH="${PATH%:}"
  unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi

# ---------------------------------------------------------------------------
# Download or use local file
# ---------------------------------------------------------------------------
TARBALL=""
CLEANUP_TARBALL=false

if [[ -n "${LOCAL_FILE}" ]]; then
  [[ -f "${LOCAL_FILE}" ]] || fail "file not found: ${LOCAL_FILE}"
  TARBALL="${LOCAL_FILE}"
  info "using local file ${DIM}${LOCAL_FILE}${NC}"
else
  DOWNLOAD_URL="${RELEASE_BASE_URL}/pnd-teleop-${ARCH}.tar.gz"
  TARBALL=$(mktemp /tmp/pnd-teleop-XXXXXX.tar.gz)
  CLEANUP_TARBALL=true

  info "downloading ${DIM}pnd-teleop-${ARCH}${NC}"

  if command -v curl &>/dev/null; then
    curl -fSL --progress-bar -o "${TARBALL}" "${DOWNLOAD_URL}" \
      || fail "download failed from ${DOWNLOAD_URL}"
  elif command -v wget &>/dev/null; then
    wget -q --show-progress -O "${TARBALL}" "${DOWNLOAD_URL}" \
      || fail "download failed from ${DOWNLOAD_URL}"
  else
    fail "curl or wget is required. Install: sudo apt install curl"
  fi
fi

# ---------------------------------------------------------------------------
# Extract
# ---------------------------------------------------------------------------
info "installing to ${DIM}${INSTALL_DIR}${NC}"
if [[ -w "$(dirname "${INSTALL_DIR}")" ]]; then
  mkdir -p "${INSTALL_DIR}"
else
  sudo mkdir -p "${INSTALL_DIR}"
  sudo chown "$(id -u):$(id -g)" "${INSTALL_DIR}"
fi
tar -xzf "${TARBALL}" -C "${INSTALL_DIR}" --strip-components=1

if [[ "${CLEANUP_TARBALL}" == "true" ]]; then
  rm -f "${TARBALL}"
fi

# ---------------------------------------------------------------------------
# Python environment
# ---------------------------------------------------------------------------
cd "${INSTALL_DIR}"

if ! command -v uv &>/dev/null; then
  info "installing uv"
  curl -LsSf https://astral.sh/uv/install.sh | sh 2>/dev/null
  export PATH="${HOME}/.local/bin:${HOME}/.cargo/bin:${PATH}"
fi

# Clean up stale venv from wrong Python version
if [[ -L .venv/bin/python ]]; then
  VENV_TARGET=$(readlink -f .venv/bin/python 2>/dev/null || true)
  if echo "${VENV_TARGET}" | grep -qE "(conda|anaconda|miniconda|miniforge)"; then
    warn "removing .venv created with conda Python"
    rm -rf .venv
  fi
fi

info "running ${DIM}uv sync${NC}"
uv sync --python 3.10 --quiet

# ---------------------------------------------------------------------------
# Verify
# ---------------------------------------------------------------------------
ERRORS=0
[[ -d install ]]   || { warn "install/ missing"; ERRORS=$((ERRORS + 1)); }
[[ -d .venv/bin ]] || { warn ".venv missing";    ERRORS=$((ERRORS + 1)); }

if [[ ${ERRORS} -gt 0 ]]; then
  warn "${ERRORS} issue(s) detected"
fi

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
VERSION=""
[[ -f .precompiled ]] && VERSION=$(cat .precompiled)

echo ""
echo -e "${GREEN}pnd-teleop${NC}${VERSION:+ ${DIM}v${VERSION}${NC}} installed successfully"
echo ""
echo -e "To get started, run:"
echo ""
echo -e "  ${BOLD}source ${INSTALL_DIR}/setup_cli.bash${NC}"
echo -e "  ${BOLD}spteleop teleop adam_u webvr mink${NC}"
echo ""
