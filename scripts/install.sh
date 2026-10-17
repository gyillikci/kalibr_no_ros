#!/usr/bin/env bash
# Install kalibr_no_ros on Ubuntu: apt dependencies, a Python venv with
# rosbags, then configure, build, install and test. Follows the README steps.
#
# Usage: scripts/install.sh [options]
#   --prefix DIR      install prefix            (default: <repo>/install)
#   --build-dir DIR   CMake build directory     (default: <repo>/build/linux-release)
#   --venv DIR        Python venv to create/use (default: ~/.venvs/kalibr)
#   --jobs N          parallel build jobs       (default: nproc)
#   --skip-apt        do not install apt packages, only check for them
#   --skip-tests      do not run ctest
#   -h, --help        show this help

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="$ROOT/install"
BUILD_DIR="$ROOT/build/linux-release"
VENV="$HOME/.venvs/kalibr"
JOBS="$(nproc)"
SKIP_APT=0
SKIP_TESTS=0

APT_PACKAGES=(
  build-essential cmake git patch
  python3-dev python3-venv python3-numpy python3-opencv python3-matplotlib
  python3-scipy python3-yaml python3-igraph python3-wxgtk4.0 python3-pil
  libboost-all-dev libopencv-dev libeigen3-dev libsuitesparse-dev libtbb-dev
  libv4l-dev
)
ROSBAGS_SPEC='rosbags>=0.9.20,<0.10'

usage() { sed -n '2,13s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; }
step() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix) PREFIX="$(realpath -m "$2")"; shift 2 ;;
    --build-dir) BUILD_DIR="$(realpath -m "$2")"; shift 2 ;;
    --venv) VENV="$(realpath -m "$2")"; shift 2 ;;
    --jobs) JOBS="$2"; shift 2 ;;
    --skip-apt) SKIP_APT=1; shift ;;
    --skip-tests) SKIP_TESTS=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "unknown option: $1" ;;
  esac
done

step "Checking platform"
. /etc/os-release
case "${VERSION_ID:-}" in
  20.04|26.04) echo "Ubuntu $VERSION_ID (validated)" ;;
  *) echo "warning: ${PRETTY_NAME:-unknown OS} is not validated; only Ubuntu 20.04 and 26.04 are." >&2 ;;
esac

step "Checking apt packages"
missing=()
for pkg in "${APT_PACKAGES[@]}"; do
  dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
done
if [[ ${#missing[@]} -eq 0 ]]; then
  echo "all ${#APT_PACKAGES[@]} packages installed"
elif [[ $SKIP_APT -eq 1 ]]; then
  die "missing apt packages: ${missing[*]}"
else
  echo "installing: ${missing[*]}"
  sudo apt-get update
  sudo apt-get install -y "${missing[@]}"
fi

# The venv is built from the system interpreter so it matches python3-dev and
# the apt OpenCV/NumPy/wx, even when another python3 (e.g. conda) is on PATH.
step "Preparing Python venv at $VENV"
if [[ ! -x "$VENV/bin/python" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/python" -m pip install --quiet "$ROSBAGS_SPEC"
"$VENV/bin/python" -c 'import numpy, cv2, wx, matplotlib, scipy, yaml, igraph, PIL, rosbags' \
  || die "the venv cannot import the required Python modules"

step "Configuring ($BUILD_DIR)"
cmake -S "$ROOT" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DPYTHON_EXECUTABLE="$VENV/bin/python"

step "Building with $JOBS jobs"
cmake --build "$BUILD_DIR" --parallel "$JOBS"

step "Installing to $PREFIX"
cmake --install "$BUILD_DIR"

if [[ $SKIP_TESTS -eq 0 ]]; then
  step "Running tests"
  (cd "$BUILD_DIR" && ctest --output-on-failure)
fi

cat <<EOF

kalibr_no_ros is installed. To use it in a new shell:

  source "$VENV/bin/activate"
  export PATH="$PREFIX/bin:\$PATH"
  kalibr_calibrate_cameras --help
EOF
