#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
THIRD_PARTY_DIR="${REPO_ROOT}/third_party"
BASALT_DIR="${THIRD_PARTY_DIR}/basalt"
BASALT_REPO_URL="https://github.com/VladyslavUsenko/basalt.git"
BASALT_COMMIT="0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
BASALT_WRAPPER_PATCH="${REPO_ROOT}/patches/basalt_global_position_factors.patch"
MIN_CMAKE_VERSION="3.24.0"

version_ge() {
  [[ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | tail -n1)" == "$1" ]]
}

ensure_modern_cmake() {
  local cmake_cmd="cmake"
  local cmake_version=""

  if command -v cmake >/dev/null 2>&1; then
    cmake_version="$(cmake --version | awk 'NR==1 {print $3}')"
  fi

  if [[ -n "${cmake_version}" ]] && version_ge "${cmake_version}" "${MIN_CMAKE_VERSION}"; then
    printf '%s' "${cmake_cmd}"
    return 0
  fi

  echo "cmake >= ${MIN_CMAKE_VERSION} is required (found '${cmake_version:-none}')." >&2
  echo "Install a newer CMake, for example with: python3 -m pip install --user 'cmake>=${MIN_CMAKE_VERSION},<4'" >&2
  exit 1
}

require_command() {
  local cmd="$1"
  local package_hint="$2"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "Missing required command '${cmd}'. Install it first (${package_hint})." >&2
    exit 1
  fi
}

mkdir -p "${THIRD_PARTY_DIR}"

CMAKE_BIN="$(ensure_modern_cmake)"
echo "Using cmake from ${CMAKE_BIN}"

require_command git "apt install git"
require_command ninja "apt install ninja-build"
require_command cc "apt install build-essential"
require_command c++ "apt install build-essential"
require_command python3 "apt install python3"

NINJA_BIN="$(command -v ninja)"
CC_BIN="$(command -v cc)"
CXX_BIN="$(command -v c++)"
BUILD_JOBS="${BASALT_BUILD_JOBS:-$(nproc)}"
if [[ "$(uname -m)" == "aarch64" || "$(uname -m)" == "arm64" ]]; then
  BUILD_JOBS="${BASALT_BUILD_JOBS:-2}"
fi

trim_wrapper_unneeded_vcpkg_deps() {
  local manifest_path="${BASALT_DIR}/vcpkg.json"
  if [[ ! -f "${manifest_path}" ]]; then
    echo "Expected Basalt manifest at ${manifest_path}" >&2
    exit 1
  fi

  python3 - "${manifest_path}" <<'PY'
import json
import sys
from pathlib import Path

manifest_path = Path(sys.argv[1])
data = json.loads(manifest_path.read_text())

def dep_name(dep):
    return dep if isinstance(dep, str) else dep.get("name")

data["dependencies"] = [
    dep for dep in data.get("dependencies", [])
    if dep_name(dep) != "realsense2"
]
data["overrides"] = [
    override for override in data.get("overrides", [])
    if override.get("name") != "realsense2"
]

manifest_path.write_text(json.dumps(data, indent=2) + "\n")
PY
}

if [[ ! -d "${BASALT_DIR}/.git" ]]; then
  git clone --recursive "${BASALT_REPO_URL}" "${BASALT_DIR}"
fi

if ! git -C "${BASALT_DIR}" cat-file -e "${BASALT_COMMIT}^{commit}" 2>/dev/null; then
  git -C "${BASALT_DIR}" fetch --tags --force origin
fi
git -C "${BASALT_DIR}" checkout --detach "${BASALT_COMMIT}"
git -C "${BASALT_DIR}" submodule update --init --recursive

if git -C "${BASALT_DIR}" apply --reverse --check "${BASALT_WRAPPER_PATCH}"; then
  echo "basalt_wrapper estimator patch is already applied"
else
  git -C "${BASALT_DIR}" apply --check "${BASALT_WRAPPER_PATCH}"
  git -C "${BASALT_DIR}" apply "${BASALT_WRAPPER_PATCH}"
  echo "Applied basalt_wrapper estimator patch"
fi

# The ROS 2 wrapper only needs Basalt core/library functionality. Upstream's
# full manifest includes device support such as realsense2, which is not
# required here and is a common source of Ubuntu build failures.
trim_wrapper_unneeded_vcpkg_deps

if [[ -x "${BASALT_DIR}/thirdparty/vcpkg/bootstrap-vcpkg.sh" ]]; then
  if [[ ! -x "${BASALT_DIR}/thirdparty/vcpkg/vcpkg" ]] \
      || ! "${BASALT_DIR}/thirdparty/vcpkg/vcpkg" version >/dev/null 2>&1; then
    rm -f "${BASALT_DIR}/thirdparty/vcpkg/vcpkg"
    "${BASALT_DIR}/thirdparty/vcpkg/bootstrap-vcpkg.sh" -disableMetrics
  fi
fi

"${CMAKE_BIN}" --preset relwithdebinfo \
  -S "${BASALT_DIR}" \
  -B "${BASALT_DIR}/build/relwithdebinfo" \
  -DCXX_MARCH="${BASALT_CXX_MARCH:-native}" \
  -DCMAKE_MAKE_PROGRAM="${NINJA_BIN}" \
  -DCMAKE_C_COMPILER="${CC_BIN}" \
  -DCMAKE_CXX_COMPILER="${CXX_BIN}"
"${CMAKE_BIN}" --build "${BASALT_DIR}/build/relwithdebinfo" -j"${BUILD_JOBS}"

echo "Basalt is ready at ${BASALT_DIR}"
