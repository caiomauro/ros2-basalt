#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
THIRD_PARTY_DIR="${REPO_ROOT}/third_party"
BASALT_DIR="${THIRD_PARTY_DIR}/basalt"
BASALT_REPO_URL="https://github.com/VladyslavUsenko/basalt.git"
BASALT_COMMIT="0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
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

  if ! command -v python3 >/dev/null 2>&1; then
    echo "cmake >= ${MIN_CMAKE_VERSION} is required and python3 is not available to bootstrap it." >&2
    exit 1
  fi

  echo "Installing a newer cmake via pip because system cmake is too old..."
  python3 -m pip install --user "cmake>=${MIN_CMAKE_VERSION},<4"

  local user_cmake="${HOME}/.local/bin/cmake"
  if [[ ! -x "${user_cmake}" ]]; then
    echo "Failed to install cmake via pip at ${user_cmake}" >&2
    exit 1
  fi

  printf '%s' "${user_cmake}"
}

mkdir -p "${THIRD_PARTY_DIR}"

CMAKE_BIN="$(ensure_modern_cmake)"
echo "Using cmake from ${CMAKE_BIN}"

if [[ ! -d "${BASALT_DIR}/.git" ]]; then
  git clone --recursive "${BASALT_REPO_URL}" "${BASALT_DIR}"
fi

git -C "${BASALT_DIR}" fetch --tags --force origin
git -C "${BASALT_DIR}" checkout "${BASALT_COMMIT}"
git -C "${BASALT_DIR}" submodule update --init --recursive

if [[ -x "${BASALT_DIR}/thirdparty/vcpkg/bootstrap-vcpkg.sh" ]] && [[ ! -x "${BASALT_DIR}/thirdparty/vcpkg/vcpkg" ]]; then
  "${BASALT_DIR}/thirdparty/vcpkg/bootstrap-vcpkg.sh" -disableMetrics
fi

"${CMAKE_BIN}" --preset relwithdebinfo -S "${BASALT_DIR}" -B "${BASALT_DIR}/build/relwithdebinfo"
"${CMAKE_BIN}" --build "${BASALT_DIR}/build/relwithdebinfo" -j"$(nproc)"

echo "Basalt is ready at ${BASALT_DIR}"
