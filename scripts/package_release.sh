#!/usr/bin/env bash
# Package a pre-compiled release tarball for pnd-teleop.
# Run this AFTER a successful `./build.sh` on the target platform.
#
# Output: pnd-teleop-<arch>.tar.gz
#
# The tarball contains everything needed to run without building from source:
#   - install/        (colcon build output)
#   - scripts/        (CLI code)
#   - src/visualization/adam_description/  (URDF & mesh placeholders)
#   - pyproject.toml, setup_cli.bash, preview.sh, run.sh
#   - quickstart.sh   (post-extract bootstrap)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

info()  { echo -e "${BLUE}[INFO]${NC} $1"; }
ok()    { echo -e "${GREEN}[OK]${NC} $1"; }
fail()  { echo -e "${RED}[ERROR]${NC} $1" >&2; exit 1; }

# ---------------------------------------------------------------------------
# Validate
# ---------------------------------------------------------------------------
[[ -d install ]]          || fail "install/ not found. Run ./build.sh first."
[[ -f pyproject.toml ]]   || fail "pyproject.toml not found."
[[ -f setup_cli.bash ]]   || fail "setup_cli.bash not found."

VERSION=$(cat "${PROJECT_ROOT}/.version" 2>/dev/null || python3 -c "
import re, pathlib
m = re.search(r'version\s*=\s*\"([^\"]+)\"', pathlib.Path('pyproject.toml').read_text())
print(m.group(1) if m else '0.0.0')
")

ARCH=$(uname -m)
case "$ARCH" in
  x86_64)  ARCH="amd64" ;;
  aarch64) ARCH="arm64" ;;
esac

RELEASE_NAME="pnd-teleop-${ARCH}"
OUT_DIR="${PROJECT_ROOT}/release"
STAGE_DIR="${OUT_DIR}/${RELEASE_NAME}"

info "Packaging release: ${RELEASE_NAME} (v${VERSION})"

# ---------------------------------------------------------------------------
# Stage
# ---------------------------------------------------------------------------
rm -rf "${STAGE_DIR}"
mkdir -p "${STAGE_DIR}"

# Core colcon output (exclude mesh & cache files — meshes are downloaded on demand by CLI)
info "Copying install/ (excluding mesh and cache files) ..."
rsync -a \
  --exclude='*.stl' --exclude='*.STL' \
  --exclude='*.dae' --exclude='*.DAE' \
  --exclude='*.obj' --exclude='*.OBJ' \
  --exclude='.cache' \
  install/ "${STAGE_DIR}/install/"

# CLI scripts
info "Copying scripts/ ..."
cp -a scripts "${STAGE_DIR}/scripts"

# URDF & description (meshes will be downloaded on demand by CLI)
info "Copying adam_description (URDF/XML only, no mesh/cache files) ..."
mkdir -p "${STAGE_DIR}/src/visualization"
rsync -a \
  --exclude='.cache' \
  --include='*/' \
  --include='*.urdf' --include='*.xml' --include='*.yaml' --include='*.yml' \
  --include='*.py' --include='*.txt' --include='*.cmake' --include='package.xml' \
  --exclude='*' \
  src/visualization/adam_description/ \
  "${STAGE_DIR}/src/visualization/adam_description/"

# Top-level files
for f in pyproject.toml setup_cli.bash preview.sh run.sh quickstart.sh build.sh README.md; do
  [[ -f "$f" ]] && cp "$f" "${STAGE_DIR}/"
done

# Bringup launch files (needed by the CLI to discover launch combinations)
if [[ -d src/bringup ]]; then
  info "Copying bringup package ..."
  mkdir -p "${STAGE_DIR}/src"
  cp -a src/bringup "${STAGE_DIR}/src/bringup"
fi

# Marker file so CLI can detect pre-compiled mode
echo "${VERSION}" > "${STAGE_DIR}/.precompiled"

# ---------------------------------------------------------------------------
# Compress
# ---------------------------------------------------------------------------
info "Creating tarball ..."
TARBALL="${OUT_DIR}/${RELEASE_NAME}.tar.gz"
tar -czf "${TARBALL}" -C "${OUT_DIR}" "${RELEASE_NAME}"
rm -rf "${STAGE_DIR}"

ok "Release packaged: ${TARBALL} (v${VERSION})"
ok "Size: $(du -h "${TARBALL}" | cut -f1)"
echo ""
info "Upload ${TARBALL} to GitHub/GitLab Releases as pnd-teleop-${ARCH}.tar.gz"
info "Users install via: curl -fsSL https://pndbotics.com/install.sh | bash"
